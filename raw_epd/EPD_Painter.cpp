#include <cstddef>
#include "esp32-hal.h"
#include <array>
#include "esp_timer.h"
#include <stdint.h>
#include "Adafruit_GFX.h"
#include "EPD_Painter.h"
#include <esp_heap_caps.h>
#include <cstring>
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>

extern "C" void epd_painter_compact_pixels(
  const uint8_t *input,
  uint8_t *output,
  uint32_t size);

extern "C" uint32_t epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb,
  uint8_t *output,
  uint32_t length,
  const uint32_t *waveform);

extern "C" uint32_t epd_painter_ink_on(
  const uint8_t *packed_src_fb,
  const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb,
  int16_t width,
  int16_t height,
  bool interlace_period);

extern "C" uint32_t epd_painter_ink_off(
  const uint8_t *packed_src_fb,
  const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb,
  int16_t width,
  int16_t height,
  bool interlace_period);

extern "C" void epd_painter_interleaved_copy(
  const uint8_t *input,
  uint8_t *output,
  int16_t width,
  int16_t height,
  bool interlace_period);

#define PASS_COUNT 6

// ---- LCD_CAM / DMA pin mapping ----
static const struct {
  int8_t pin;
  uint8_t signal;
} kDataPins[] = {
  { 6, LCD_DATA_OUT0_IDX },
  { 14, LCD_DATA_OUT1_IDX },
  { 7, LCD_DATA_OUT2_IDX },
  { 12, LCD_DATA_OUT3_IDX },
  { 9, LCD_DATA_OUT4_IDX },
  { 11, LCD_DATA_OUT5_IDX },
  { 8, LCD_DATA_OUT6_IDX },
  { 10, LCD_DATA_OUT7_IDX },
};

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
EPD_Painter::EPD_Painter()
  : GFXcanvas8(960, 540, false) {

  uint32_t bytes = (uint32_t)width() * (uint32_t)height();
  buffer = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  if (buffer) memset(buffer, 0, bytes);
}

// -----------------------------------------------------------------------
// sendRow() — DMA-transfer one packed row to the EPD via LCD_CAM
// -----------------------------------------------------------------------

void EPD_Painter::sendRow(bool firstLine, bool lastLine) {
  //

  // swap buffers
  dma_buffer = dma_buffer == dma_buffer1 ? dma_buffer2 : dma_buffer1;

  while (LCD_CAM.lcd_user.lcd_start) {
    yield();
  }

  if (firstLine) {
    // Move to the start row
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_SPV) | (1 << PIN_CKV));
    delayMicroseconds(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_SPV));
  } else {
     // Latch row and advance row clock
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_LE));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_CKV));
    delayMicroseconds(6);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_LE));
  }

  // Start DMA transfer
  LCD_CAM.lcd_user.lcd_start = 1;

  // 
  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {
      yield();
    }

    // Latch row and advance row clock
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_LE));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_CKV));
    delayMicroseconds(5);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_LE));
  }
}

// -----------------------------------------------------------------------
// begin()
// -----------------------------------------------------------------------
bool EPD_Painter::begin() {
  // ---- GPIO setup ----
  pinMode(PIN_PWR, OUTPUT);
  pinMode(PIN_SPV, OUTPUT);
  pinMode(PIN_CKV, OUTPUT);
  pinMode(PIN_SPH, OUTPUT);
  pinMode(PIN_OE, OUTPUT);
  pinMode(PIN_LE, OUTPUT);
  pinMode(PIN_CL, OUTPUT);

  packed_row_bytes = width() / 4;

  // ---- Enable and reset LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);

  // ---- Clock: PLL_D2 (~80 MHz) / 2 / 2 = 20 MHz ----
  // Adjust lcd_clkm_div_num / lcd_clkcnt_n to suit your panel's max CL rate
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;         // PLL_F160M_CLK
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;     // PCLK low in first half-cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;    // PCLK low when idle
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;  // Use divider
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 1;    // ÷2 → 40 MHz
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;  // ÷2 → 20 MHz

  // ---- Frame format: i8080, 8-bit, no conversions ----
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  LCD_CAM.lcd_data_dout_mode.val = 0;
  LCD_CAM.lcd_user.lcd_always_out_en = 0;
  LCD_CAM.lcd_user.lcd_8bits_order = 0;
  LCD_CAM.lcd_user.lcd_bit_order = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = 0;
  LCD_CAM.lcd_user.lcd_dummy = 0;
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
  LCD_CAM.lcd_user.lcd_cmd = 0;

  LCD_CAM.lcd_user.lcd_dout_cyclelen = packed_row_bytes - 1;  // 0-indexed
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;


  // ---- Connect 8-bit data bus to GPIO pins ----
  for (int i = 0; i < 8; i++) {
    esp_rom_gpio_connect_out_signal(kDataPins[i].pin, kDataPins[i].signal, false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[kDataPins[i].pin], PIN_FUNC_GPIO);
    gpio_set_drive_capability((gpio_num_t)kDataPins[i].pin, (gpio_drive_cap_t)3);
  }

  // ---- Connect pixel clock to CL pin ----
  esp_rom_gpio_connect_out_signal(PIN_CL, LCD_PCLK_IDX, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_CL], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)PIN_CL, (gpio_drive_cap_t)3);

  // ---- Configure DMA channel → LCD peripheral ----
  gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = { .reserve_sibling = 0 },
  };
  gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false,
  };
  gdma_apply_strategy(dma_chan, &strategy_config);

  // ---- Allocate DMA row buffer (internal RAM, DMA-capable) ----
  dma_buffer1 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

  dma_buffer = dma_buffer1;

  // ---- DMA descriptor (size/buffer filled per transfer in sendRow) ----
  dma_desc2.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc2.dw0.suc_eof = 0;
  dma_desc2.dw0.size = packed_row_bytes;
  dma_desc2.dw0.length = packed_row_bytes;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2);


  dma_desc1.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc1.dw0.suc_eof = 0;
  dma_desc1.dw0.size = packed_row_bytes;
  dma_desc1.dw0.length = packed_row_bytes;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1);

  dma_desc1.next = &dma_desc2;
  dma_desc2.next = &dma_desc1;

  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)&dma_desc1);


  // ---- Allocate packed framebuffers ----
  const size_t packed_size = (width() * height()) / 4;  // 2 bpp

  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  memset(packed_screenbuffer, 0x00, packed_size);
  memset(getBuffer(), 0x00, width() * height());

  return (dma_buffer && packed_fastbuffer && packed_screenbuffer);
}

// -----------------------------------------------------------------------
// end()
// -----------------------------------------------------------------------
bool EPD_Painter::end() {
  if (dma_chan) {
    gdma_disconnect(dma_chan);
    gdma_del_channel(dma_chan);
    dma_chan = nullptr;
  }
  periph_module_disable(PERIPH_LCD_CAM_MODULE);
  return true;
}

// -----------------------------------------------------------------------
// Power control
// -----------------------------------------------------------------------
void EPD_Painter::powerOn() {
  digitalWrite(PIN_SPV, LOW);
  digitalWrite(PIN_SPH, LOW);
  digitalWrite(PIN_OE, HIGH);
  delayMicroseconds(100);
  digitalWrite(PIN_PWR, HIGH);
  delayMicroseconds(100);

  // Frame start pulse — position at top of screen
  REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_SPV) | (1 << PIN_CKV));
  delayMicroseconds(1);
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_SPV));
}

void EPD_Painter::powerOff() {
  digitalWrite(PIN_PWR, LOW);
  delayMicroseconds(100);
  digitalWrite(PIN_OE, LOW);
}

// -----------------------------------------------------------------------
// Waveforms
// -----------------------------------------------------------------------
static const uint8_t lighter_waveform[][6] = {
  { 1, 2, 2, 2, 3, 0 },
  { 1, 2, 2, 2, 3, 2 },
  { 2, 2, 2, 2, 2, 2 }
};

static const uint8_t darker_waveform[][6] = {
  { 1, 1, 0, 0, 0, 0 },
  { 1, 1, 1, 0, 0, 0 },
  { 1, 1, 1, 1, 1, 1 }
};

// -----------------------------------------------------------------------
// paint()
// -----------------------------------------------------------------------
void EPD_Painter::paint() {
  PanelPowerGuard guard(*this);
  const int packed_row_bytes = width() / 4;

  epd_painter_compact_pixels(buffer, packed_fastbuffer, width() * height());

  epd_painter_ink_on( packed_fastbuffer, packed_screenbuffer, packed_fastbuffer, packed_row_bytes, height(),  interlace_period);
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer, packed_row_bytes, height(), !interlace_period);

  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {
    const uint32_t lighter_fmt =
      ((lighter_waveform[0][pass] << 16) + (lighter_waveform[1][pass] << 8) + lighter_waveform[2][pass]) * 0x55;
    const uint32_t darker_fmt =
      ((darker_waveform[0][pass] << 16)  + (darker_waveform[1][pass]  << 8) + darker_waveform[2][pass])  * 0x55;

    for (int row = 0; row < height(); row++) {
      uint32_t waveform = (row % 2 == interlace_period) ? darker_fmt : lighter_fmt;
      epd_painter_convert_packed_fb_to_ink( packed_fastbuffer + row * packed_row_bytes, dma_buffer, packed_row_bytes, &waveform);

  //    int start = esp_timer_get_time();
       sendRow(row==0);
//      Serial.println(esp_timer_get_time()-start);
    }
  }

  // Neutralise
  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < height(); ++row) {
    sendRow(row==0, row==height()-1);
  }

  interlace_period = !interlace_period;
}

// -----------------------------------------------------------------------
// clear()
// -----------------------------------------------------------------------
void EPD_Painter::clear() {
  PanelPowerGuard guard(*this);
  const int packed_row_bytes = width() / 4;

  memset(packed_fastbuffer, 0x00, height() * packed_row_bytes);
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer, packed_row_bytes, height(), true);
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer, packed_row_bytes, height(), false);

  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {
    const uint32_t lighter_fmt = ((lighter_waveform[0][pass] << 16) + (lighter_waveform[1][pass] << 8) + lighter_waveform[2][pass]) * 0x55;
    for (int row = 0; row < height(); row++) {
      epd_painter_convert_packed_fb_to_ink( packed_fastbuffer + row * packed_row_bytes, dma_buffer, packed_row_bytes, &lighter_fmt);
      sendRow(row==0);
    }
  }

  // Erase: alternate white/black drives
  for (int phase = 0; phase < 2; phase++) {
    memset(dma_buffer1, (phase % 2 == 0) ? 0b01010101 : 0b10101010, packed_row_bytes);
    memset(dma_buffer2, (phase % 2 == 0) ? 0b01010101 : 0b10101010, packed_row_bytes);

    for (int passes=0; passes<4; passes++){
      for (int row = 0; row < height(); ++row) {
        sendRow(row==0);
      }
    }
  }

  // Neutralise
  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < height(); ++row) {
    sendRow(row==0, row==height()-1);
  }
  delay(1);

}