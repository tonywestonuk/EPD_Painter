#include "esp32-hal.h"
#include "esp_timer.h"
#include "Adafruit_GFX.h"
#include "EPD_Painter.h"
#include <esp_heap_caps.h>
#include <cstring>
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>

// LCD_CAM signal indices for the 8 parallel data lines
static const uint8_t kDataSignals[8] = {
  LCD_DATA_OUT0_IDX,
  LCD_DATA_OUT1_IDX,
  LCD_DATA_OUT2_IDX,
  LCD_DATA_OUT3_IDX,
  LCD_DATA_OUT4_IDX,
  LCD_DATA_OUT5_IDX,
  LCD_DATA_OUT6_IDX,
  LCD_DATA_OUT7_IDX,
};

// Assembly routines — see EPD_Painter.S for full documentation
extern "C" void epd_painter_compact_pixels(
  const uint8_t *input, uint8_t *output, uint32_t size);

extern "C" uint32_t epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint32_t *waveform);

extern "C" uint32_t epd_painter_ink_on(
  const uint8_t *packed_src_fb, const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb, int16_t width, int16_t height, bool interlace_period);

extern "C" uint32_t epd_painter_ink_off(
  const uint8_t *packed_src_fb, const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb, int16_t width, int16_t height, bool interlace_period);

extern "C" void epd_painter_interleaved_copy(
  const uint8_t *input, uint8_t *output,
  int16_t width, int16_t height, bool interlace_period);
  

#define PASS_COUNT 6

// =============================================================================
// Constructor — dimensions come from config defaults until begin() is called.
// We construct GFXcanvas8 with 0,0 and resize in begin() once we have config.
// =============================================================================
EPD_Painter::EPD_Painter(const Config &config)
  : GFXcanvas8(config.width, config.height, false) {
  
    _config = config;
}

void EPD_Painter::setQuality(Quality quality){
      switch (quality) {
        case Quality::QUALITY_HIGH:
            _config.latch_delay=25;
            break;
        case Quality::QUALITY_NORMAL:
            _config.latch_delay=10;
            break;
        case Quality::QUALITY_FAST:
            _config.latch_delay=5;
            // e.g. fewer passes, faster but ghostier
            break;
    }
}

// =============================================================================
// sendRow()
// =============================================================================
void EPD_Painter::sendRow(bool firstLine, bool lastLine) {
  dma_buffer = dma_buffer == dma_buffer1 ? dma_buffer2 : dma_buffer1;

  while (LCD_CAM.lcd_user.lcd_start) {
    yield();
  }

  if (firstLine) {
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_spv) | (1 << _config.pin_ckv));
    delayMicroseconds(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_ckv));
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_spv));
  } else {
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_le));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_ckv));
    delayMicroseconds(_config.latch_delay);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_ckv));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_le));
  }

  LCD_CAM.lcd_user.lcd_start = 1;

  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {
      yield();
    }
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_le));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_ckv));
    delayMicroseconds(_config.latch_delay);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_ckv));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_le));
  }
}

// =============================================================================
// begin()
// =============================================================================
bool EPD_Painter::begin() {

  // ---- Allocate GFX canvas buffer now we know dimensions ----
  uint32_t bytes = (uint32_t)_config.width * (uint32_t)_config.height;
  buffer = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  if (buffer) memset(buffer, 0, bytes);
  

  // ---- Configure EPD control pins ----
  pinMode(_config.pin_pwr, OUTPUT);
  pinMode(_config.pin_spv, OUTPUT);
  pinMode(_config.pin_ckv, OUTPUT);
  pinMode(_config.pin_sph, OUTPUT);
  pinMode(_config.pin_oe,  OUTPUT);
  pinMode(_config.pin_le,  OUTPUT);
  pinMode(_config.pin_cl,  OUTPUT);

  packed_row_bytes = _config.width / 4;

  // ---- Enable and reset LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);

  // ---- Configure LCD_CAM pixel clock ----
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 1;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;

  // ---- Configure LCD_CAM for i8080 8-bit parallel mode ----
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
  LCD_CAM.lcd_user.lcd_dout_cyclelen = packed_row_bytes - 1;
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;

  // ---- Route 8-bit data bus pins using config ----
  for (int i = 0; i < 8; i++) {
    int8_t pin = _config.data_pins[i];
    esp_rom_gpio_connect_out_signal(pin, kDataSignals[i], false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
    gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
  }

  // ---- Route pixel clock to CL pin ----
  esp_rom_gpio_connect_out_signal(_config.pin_cl, LCD_PCLK_IDX, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[_config.pin_cl], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)_config.pin_cl, (gpio_drive_cap_t)3);

  // ---- Allocate GDMA channel ----
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

  // ---- Allocate DMA row buffers ----
  dma_buffer1 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

  dma_buffer = dma_buffer1;

  // ---- Set up circular DMA descriptor chain ----
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

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = (_config.width * _config.height) / 4;

  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  memset(packed_screenbuffer, 0x00, packed_size);
  memset(getBuffer(), 0x00, _config.width * _config.height);

  return (dma_buffer && packed_fastbuffer && packed_screenbuffer);
}

// =============================================================================
// end()
// =============================================================================
bool EPD_Painter::end() {
  if (dma_chan) {
    gdma_disconnect(dma_chan);
    gdma_del_channel(dma_chan);
    dma_chan = nullptr;
  }
  periph_module_disable(PERIPH_LCD_CAM_MODULE);
  return true;
}

// =============================================================================
// Power control
// =============================================================================
void EPD_Painter::powerOn() {
  digitalWrite(_config.pin_spv, LOW);
  digitalWrite(_config.pin_sph, LOW);
  digitalWrite(_config.pin_oe,  HIGH);
  delayMicroseconds(100);
  digitalWrite(_config.pin_pwr, HIGH);
  delayMicroseconds(100);

  REG_WRITE(GPIO_OUT_W1TC_REG, (1 << _config.pin_spv) | (1 << _config.pin_ckv));
  delayMicroseconds(1);
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_ckv));
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << _config.pin_spv));
}

void EPD_Painter::powerOff() {
  digitalWrite(_config.pin_pwr, LOW);
  delayMicroseconds(100);
  digitalWrite(_config.pin_oe,  LOW);
}

// =============================================================================
// Waveform tables
// =============================================================================
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

// =============================================================================
// paint()
// =============================================================================
void EPD_Painter::paint() {
  PanelPowerGuard guard(*this);
  const int packed_row_bytes = width() / 4;


  epd_painter_compact_pixels(buffer, packed_fastbuffer, width()*height());

  epd_painter_ink_on(
    packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
    packed_row_bytes, height(), interlace_period);

  epd_painter_ink_off(
    packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
    packed_row_bytes, height(), !interlace_period);

  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {
    const uint32_t lighter_fmt =
      ((lighter_waveform[0][pass] << 16) +
       (lighter_waveform[1][pass] << 8)  +
        lighter_waveform[2][pass]) * 0x55;

    const uint32_t darker_fmt =
      ((darker_waveform[0][pass] << 16) +
       (darker_waveform[1][pass] << 8)  +
        darker_waveform[2][pass]) * 0x55;

    for (int row = 0; row < height(); row++) {
      uint32_t waveform = (row % 2 == interlace_period) ? darker_fmt : lighter_fmt;
      epd_painter_convert_packed_fb_to_ink(
        packed_fastbuffer + row * packed_row_bytes,
        dma_buffer, packed_row_bytes, &waveform);
      sendRow(row == 0);
    }
  }

  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < height(); ++row) {
    sendRow(row == 0, row == height() - 1);
  }

  interlace_period = !interlace_period;
}

// =============================================================================
// clear()
// =============================================================================
void EPD_Painter::clear() {
  PanelPowerGuard guard(*this);
  const int packed_row_bytes = width() / 4;

  memset(packed_fastbuffer, 0x00, height() * packed_row_bytes);

  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
                      packed_row_bytes, height(), true);
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
                      packed_row_bytes, height(), false);

  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {
    const uint32_t lighter_fmt =
      ((lighter_waveform[0][pass] << 16) +
       (lighter_waveform[1][pass] << 8)  +
        lighter_waveform[2][pass]) * 0x55;

    for (int row = 0; row < height(); row++) {
      epd_painter_convert_packed_fb_to_ink(
        packed_fastbuffer + row * packed_row_bytes,
        dma_buffer, packed_row_bytes, &lighter_fmt);
      sendRow(row == 0);
    }
  }

  for (int phase = 0; phase < 2; phase++) {
    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;
    memset(dma_buffer1, pattern, packed_row_bytes);
    memset(dma_buffer2, pattern, packed_row_bytes);

    for (int passes = 0; passes < 4; passes++) {
      for (int row = 0; row < _config.height; ++row) {
        sendRow(row == 0);
      }
    }
  }

  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < _config.height; ++row) {
    sendRow(row == 0, row == _config.height - 1);
  }
}