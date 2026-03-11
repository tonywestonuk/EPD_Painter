#include "esp32-hal.h"
#include <string.h>
#include <cstring>
#include "build_opt.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>
#include "EPD_Painter.h"
#include <epd_painter_powerctl.h>

#ifdef ARDUINO
  #include "Wire.h"
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
  #include "driver/i2c_master.h"
#endif

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



epd_painter_powerctl *powerctl = nullptr;

// Assembly routines — see EPD_Painter.S for full documentation
extern "C" void epd_painter_compact_pixels(
  const uint8_t *input, uint8_t *output, uint32_t size);

extern "C" void epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint8_t *waveform, uint32_t chunk_flags);

extern "C" uint32_t epd_painter_ink_on(
  uint8_t *packed_fastbuffer,
  const uint8_t *packed_screenbuffer,
  uint32_t length_bytes);

extern "C" void epd_painter_ink_off(
  uint8_t *packed_fastbuffer,
  uint8_t *packed_screenbuffer,
  uint32_t length_bytes,
  uint32_t bitmask);

extern "C" void epd_painter_interleaved_copy(
  const uint8_t *input, uint8_t *output,
  int16_t width, int16_t height, bool interlace_period);

extern "C" uint32_t epd_painter_ink(uint8_t *packed_fastbuffer, uint8_t *packed_screenbuffer, uint32_t length, uint32_t bitmask);

static inline void epd_gpio_func_sel(int pin) {
  esp_rom_gpio_pad_select_gpio((gpio_num_t)pin);
}

static inline void gpio_set_fast(uint8_t pin) {
  if (pin < 32) {
    REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << pin);
  } else {
    REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
  }
}

static inline void gpio_clear_fast(uint8_t pin) {
  if (pin < 32) {
    REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << pin);
  } else {
    REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
  }
}

#define PASS_COUNT 13


EPD_Painter::EPD_Painter(const Config &config) {
  _config = config;
}


void EPD_Painter::setQuality(Quality quality) {
  _config.quality = quality;
}

// =============================================================================
// sendRow()
// =============================================================================
void EPD_Painter::sendRow(bool firstLine, bool lastLine, bool skipRow) {

  // Wait for LCD peripheral to finish consuming the previous row.
  // This also guarantees the previously-started DMA transfer is complete,
  // since the LCD FIFO cannot drain faster than DMA fills it.
  while (LCD_CAM.lcd_user.lcd_start) {
    EPD_YIELD();
  }

  // dma_buffer points at the buffer the CPU just finished writing.
  // Start DMA on its matching descriptor, then swap so the next
  // convert_packed_fb_to_ink() writes into the now-idle buffer.
  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) {
    desc = &dma_desc1;
    dma_buffer = dma_buffer2;
  } else {
    desc = &dma_desc2;
    dma_buffer = dma_buffer1;
  }

  if (firstLine) {
    gpio_clear_fast(_config.pin_spv);
    gpio_clear_fast(_config.pin_ckv);
    gdma_start(dma_chan, (intptr_t)desc);
    gpio_set_fast(_config.pin_ckv);
    gpio_set_fast(_config.pin_spv);
  } else {
    gpio_clear_fast(_config.pin_ckv);
    gpio_set_fast(_config.pin_le);
    gdma_start(dma_chan, (intptr_t)desc);
    gpio_clear_fast(_config.pin_le);
    gpio_set_fast(_config.pin_ckv);
  }

  LCD_CAM.lcd_user.lcd_start = 1;

  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {
      EPD_YIELD();
    }
    gpio_clear_fast(_config.pin_ckv);
    gpio_set_fast(_config.pin_le);
    EPD_DELAY_US(1);
    gpio_clear_fast(_config.pin_le);
    gpio_set_fast(_config.pin_ckv);
  }
}

// =============================================================================
// begin()
// =============================================================================
bool EPD_Painter::begin() {

  // -- Start I2C if needed.
#ifdef ARDUINO
  if (_config.i2c.scl != -1 && _config.i2c.wire == nullptr) {
    TwoWire *w = new TwoWire(0);
    w->begin(_config.i2c.sda, _config.i2c.scl, _config.i2c.freq);
    _config.i2c.wire = w;
    EPD_DELAY_MS(50);
  }
#else
  if (_config.i2c.scl != -1 && _config.i2c.i2c_bus == nullptr) {
    i2c_master_bus_config_t i2c_bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)_config.i2c.sda,
      .scl_io_num = (gpio_num_t)_config.i2c.scl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = { .enable_internal_pullup = true },
    };
    i2c_master_bus_handle_t bus;
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &bus);
    if (err != ESP_OK) {
      printf("EPD_Painter: I2C bus init failed (%d)\n", err);
      return false;
    }
    _config.i2c.i2c_bus = bus;
    EPD_DELAY_MS(50);
  }
#endif


  // ---- Configure EPD control pins ----
  EPD_PIN_OUTPUT(_config.pin_pwr);
  EPD_PIN_OUTPUT(_config.pin_spv);
  EPD_PIN_OUTPUT(_config.pin_ckv);
  EPD_PIN_OUTPUT(_config.pin_sph);
  EPD_PIN_OUTPUT(_config.pin_oe);
  EPD_PIN_OUTPUT(_config.pin_le);
  EPD_PIN_OUTPUT(_config.pin_cl);


  packed_row_bytes = _config.width / 4;

  // ---- Enable and reset LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  EPD_DELAY_US(100);

  // ---- Configure LCD_CAM pixel clock ----
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 2;
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
    epd_gpio_func_sel(GPIO_PIN_MUX_REG[pin]);
    gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
  }

  // ---- Route pixel clock to CL pin ----
  esp_rom_gpio_connect_out_signal(_config.pin_cl, LCD_PCLK_IDX, false, false);
  epd_gpio_func_sel(GPIO_PIN_MUX_REG[_config.pin_cl]);
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
  dma_desc2.dw0.suc_eof = 1;
  dma_desc2.dw0.size = packed_row_bytes;
  dma_desc2.dw0.length = packed_row_bytes;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2);

  dma_desc1.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc1.dw0.suc_eof = 1;
  dma_desc1.dw0.size = packed_row_bytes;
  dma_desc1.dw0.length = packed_row_bytes;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1);

  dma_desc1.next = nullptr;
  dma_desc2.next = nullptr;

  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)&dma_desc1);

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = (_config.width * _config.height) / 4;

  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  packed_paintbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  bitmask = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_SPIRAM));

  // ── If a TPS chip is present, initialise the power controller ──
  if (_config.power.tps_addr != -1) {
    printf("\n── PowerCtl Init ──\n");
    powerctl = new epd_painter_powerctl();
    if (!powerctl->begin(_config)) {
      printf("FATAL: powerctl init failed!\n");
      while (1) EPD_DELAY_MS(1000);
    }
  }

  if (!(dma_buffer && packed_fastbuffer && packed_screenbuffer)) return false;

  _paint_active_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_active_sem); 
  _paint_buffer_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_buffer_sem); 

  xTaskCreatePinnedToCore(
    _paint_task_entry, "epd_paint", 8000, this, 10, &_paint_task_h, 0);

  return true;
}

// =============================================================================
// end()
// =============================================================================
bool EPD_Painter::end() {
  if (_paint_task_h) {
    vTaskDelete(_paint_task_h);
    _paint_task_h = nullptr;
  }

  vSemaphoreDelete(_paint_active_sem);
  vSemaphoreDelete(_paint_buffer_sem);

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
  EPD_PIN_LOW(_config.pin_spv);
  EPD_PIN_LOW(_config.pin_sph);

  if (powerctl) {
    powerctl->powerOn();
  } else {
    EPD_PIN_HIGH(_config.pin_oe);
    EPD_DELAY_US(100);
    EPD_PIN_HIGH(_config.pin_pwr);
    EPD_DELAY_US(100);
  }

  gpio_clear_fast(_config.pin_spv);
  gpio_clear_fast(_config.pin_ckv);
  EPD_DELAY_US(1);

  gpio_set_fast(_config.pin_ckv);
  gpio_set_fast(_config.pin_spv);
}

void EPD_Painter::powerOff() {
  if (powerctl) {
    powerctl->powerOff();
  } else {
    EPD_PIN_LOW(_config.pin_oe);
    EPD_DELAY_US(100);
    EPD_PIN_LOW(_config.pin_pwr);
    EPD_DELAY_US(100);
  }
}

// // =============================================================================
// // Waveform tables
// // =============================================================================
static uint8_t lighter_waveform[][7] = {
  { 1, 2, 2, 0, 2, 0, 2 },
  { 2, 2, 2, 2, 2, 3, 3 },
  { 2, 2, 2, 2, 2, 2, 2 }
};

static uint8_t darker_waveform[][7] = {
  { 1, 3, 3, 3, 1, 3, 1 },
  { 1, 1, 3, 1, 1, 3, 1 },
  { 1, 1, 1, 1, 1, 1, 1 }
};


// Higer quality waveforms
static uint8_t hq_lighter_waveform[][13] = {
  { 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3 },
  { 1, 1, 2, 2, 3, 2, 2, 2, 3, 3, 2, 2, 2 },
  { 2, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 3, 2 }
};

static uint8_t hq_darker_waveform[][13] = {
  { 1, 1, 3, 3, 1, 3, 3, 3, 1, 3, 1, 2, 2 },
  { 3, 3, 3, 1, 1, 3, 3, 1, 1, 3, 1, 3, 3 },
  { 1, 1, 1, 1, 1, 3, 1, 1, 1, 1, 1, 3, 1 }
};


// =============================================================================
// paint()
// =============================================================================
void EPD_Painter::paint(uint8_t *framebuffer) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY); 
  epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
  paintStage=2;
  xSemaphoreGive(_paint_buffer_sem); 

  // wait until this buffer has been picked up by the paint loop.
  while(paintStage==2){
      vTaskDelay(1);
  }
}

// =============================================================================
// paintLater
// =============================================================================
void EPD_Painter::paintLater(uint8_t *framebuffer) {
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY); 
    epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
    paintStage=2;
    xSemaphoreGive(_paint_buffer_sem); 
}

// =============================================================================
// _paint_task_entry() / _paint_task_body()
// =============================================================================
void EPD_Painter::_paint_task_entry(void *arg) {
  static_cast<EPD_Painter *>(arg)->_paint_task_body();
}

void EPD_Painter::_paint_task_body() {
  for (;;) {
    if (paintStage==0){
      xSemaphoreGive(_paint_active_sem);
      while(paintStage==0){
         vTaskDelay(1);
      }
      xSemaphoreTake(_paint_active_sem, portMAX_DELAY);
    }

    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    memcpy(packed_fastbuffer, packed_paintbuffer, packed_row_bytes*_config.height);
    xSemaphoreGive(_paint_buffer_sem);

    paintStage-=1;
  
    PanelPowerGuard guard(*this);

    for (int row = 0; row < _config.height; row++) {
      uint8_t *fb_row = packed_fastbuffer + row * packed_row_bytes;
      uint8_t *sb_row = packed_screenbuffer + row * packed_row_bytes;
      bitmask[row] = epd_painter_ink(fb_row, sb_row, packed_row_bytes,  0xffffffff);
    }

    const uint8_t *lt_wf;
    const uint8_t *dk_wf;
    int wf_len;

    if (_config.quality == Quality::QUALITY_HIGH) {
      lt_wf = &hq_lighter_waveform[0][0];
      dk_wf = &hq_darker_waveform[0][0];
      wf_len = 13;
    } else {
      lt_wf = &lighter_waveform[0][0];
      dk_wf = &darker_waveform[0][0];
      wf_len = 7;
    }

    for (uint8_t pass = 0; pass < wf_len; pass++) {
      uint8_t lighter_wf[3] = {
        (uint8_t)(lt_wf[2 * wf_len + pass] * 0x55),
        (uint8_t)(lt_wf[1 * wf_len + pass] * 0x55),
        (uint8_t)(lt_wf[0 * wf_len + pass] * 0x55)
      };
      uint8_t darker_wf[3] = {
        (uint8_t)(dk_wf[2 * wf_len + pass] * 0x55),
        (uint8_t)(dk_wf[1 * wf_len + pass] * 0x55),
        (uint8_t)(dk_wf[0 * wf_len + pass] * 0x55)
      };

      for (int row = 0; row < _config.height; row++) {
        uint8_t *fb_row = packed_fastbuffer + row * packed_row_bytes;
        epd_painter_convert_packed_fb_to_ink(fb_row, dma_buffer, packed_row_bytes, darker_wf, bitmask[row]);
        epd_painter_convert_packed_fb_to_ink(fb_row, dma_buffer, packed_row_bytes, lighter_wf, ~bitmask[row]);
        sendRow(row == 0, false, false);
      }

      if (_config.quality == Quality::QUALITY_FAST) {
        EPD_DELAY_MS(1);
      } else if (_config.quality == Quality::QUALITY_NORMAL) {
        EPD_DELAY_MS(4);
      } else {
        EPD_DELAY_MS(6);
      }
    }

    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    for (int row = 0; row < _config.height; ++row) {
      sendRow(row == 0, row == _config.height - 1);
    }

  }
}

// =============================================================================
// clear()
// =============================================================================
void EPD_Painter::clear() {

  const int packed_row_bytes = _config.width / 4;

  // First paint it white.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY); 
  memset(packed_paintbuffer, 0x00, packed_row_bytes * _config.height);
  paintStage=1;  /// only needs 1 pass.
  xSemaphoreGive(_paint_buffer_sem); 

  while (paintStage==1){
    vTaskDelay(1);  // Wait until paintloop starts up again
  }

  // Wait until paintloop is idle.. By taking it, prevents paint loop from starting again.
  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  PanelPowerGuard guard(*this);
  const uint8_t *lt_wf;
  int wf_len;

  if (_config.quality == Quality::QUALITY_HIGH) {
    lt_wf = &hq_lighter_waveform[0][0];
    wf_len = 13;
  } else {
    lt_wf = &lighter_waveform[0][0];
    wf_len = 7;
  }

  // Send clear
  for (int phase = 0; phase < 4; phase++) {
    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;
    memset(dma_buffer1, pattern, packed_row_bytes);
    memset(dma_buffer2, pattern, packed_row_bytes);

    int totpass[] = { 6, 2, 4, 8, 6, 6, 6, 6 };

    for (int passes = 0; passes < totpass[phase]; passes++) {
      for (int row = 0; row < _config.height; ++row) {
        sendRow(row == 0);
      }
      EPD_DELAY_MS(15);
    }
  }

  // Send neutral..
  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < _config.height; ++row) {
    sendRow(row == 0, row == _config.height - 1);
  }

  
  xSemaphoreGive(_paint_active_sem); 


}