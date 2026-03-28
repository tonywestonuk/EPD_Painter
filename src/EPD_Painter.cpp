#ifdef ARDUINO
#include "esp32-hal.h"
#endif
#include <string.h>
#include <cstring>
#include "build_opt.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>
#include <soc/gdma_struct.h>
#include "EPD_Painter.h"
#include <epd_painter_powerctl.h>
#include "epd_painter_bootctl.h"

#ifdef ARDUINO
  #include "Wire.h"
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
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
epd_painter_powerctl_74HCT4094D *shiftctl = nullptr;
uint8_t shiftReg = 0;

// Assembly routines — see EPD_Painter.S for full documentation
extern "C" void epd_painter_compact_pixels(
  const uint8_t *input, uint8_t *output, uint32_t size);

// =============================================================================
// compact_pixels_rotated_cw
//
// Combines 90° clockwise rotation and 8bpp→2bpp compaction in one pass.
//
// The portrait canvas is src_w wide × src_h tall (e.g. 540×960).
// The physical panel is src_h wide × src_w tall (e.g. 960×540).
//
// Rotation mapping (clockwise):
//   canvas pixel (col=x, row=y)  →  panel pixel (col=y, row=src_w-1-x)
//
// Processed in blocks of 16 portrait rows so the working set (16 × src_w bytes)
// stays warm in cache across all src_w column iterations within each block,
// avoiding repeated PSRAM fetches for the strided column access pattern.
//
// src_h must be a multiple of 16. src_w must be a multiple of 4.
// =============================================================================
static IRAM_ATTR void compact_pixels_rotated_cw(
    const uint8_t* src, uint8_t* dst, int src_w, int src_h)
{

    const int out_stride = src_h / 4;   // packed bytes per output row (e.g. 240)

    for (int rb = 0; rb < src_h; rb += 16) {
        for (int cx = 0; cx < src_w; cx++) {
            // CW: canvas column cx → output row (src_w - 1 - cx)
            uint8_t* out       = dst + (src_w - 1 - cx) * out_stride + rb / 4;
            const uint8_t* col = src + rb * src_w + cx;

            // 16 portrait rows → 4 packed output bytes, 4 pixels per byte.
            // Fully unrolled; all 16 loads are to addresses within the current
            // 16-row block (rb * src_w .. (rb+15) * src_w), which is warm in cache.
            out[0] = ((col[ 0        ] & 3) << 6) | ((col[  src_w] & 3) << 4)
                   | ((col[ 2*src_w  ] & 3) << 2) |  (col[ 3*src_w] & 3);
            out[1] = ((col[ 4*src_w  ] & 3) << 6) | ((col[ 5*src_w] & 3) << 4)
                   | ((col[ 6*src_w  ] & 3) << 2) |  (col[ 7*src_w] & 3);
            out[2] = ((col[ 8*src_w  ] & 3) << 6) | ((col[ 9*src_w] & 3) << 4)
                   | ((col[10*src_w  ] & 3) << 2) |  (col[11*src_w] & 3);
            out[3] = ((col[12*src_w  ] & 3) << 6) | ((col[13*src_w] & 3) << 4)
                   | ((col[14*src_w  ] & 3) << 2) |  (col[15*src_w] & 3);
        }
    }

}

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


EPD_Painter::EPD_Painter(const Config &config, bool portrait) {
  _config = config;
  if (portrait) _config.rotation = Rotation::ROTATION_CW;
}


void EPD_Painter::setQuality(Quality quality) {
  _config.quality = quality;
}

// =============================================================================
// sendRow()
// =============================================================================
void EPD_Painter::sendRow(bool firstLine, bool lastLine, bool skipRow) {
  (void)skipRow;
  // Wait for LCD peripheral to finish consuming the previous row.
  // This also guarantees the previously-started DMA transfer is complete,
  // since the LCD FIFO cannot drain faster than DMA fills it.
  //long count=0;
  while (LCD_CAM.lcd_user.lcd_start) {}
  //printf("yielded %d \n",count);
  //delayMicroseconds(4);

  // dma_buffer points at the buffer the CPU just finished writing.
  // Start DMA on its matching descriptor, then swap so the next
  // convert_packed_fb_to_ink() writes into the now-idle buffer.
  //delayMicroseconds(10);
  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) {
    desc = &dma_desc1;
    dma_buffer = dma_buffer2;
  } else {
    desc = &dma_desc2;
    dma_buffer = dma_buffer1;
  }

  if (firstLine) {
    if(shiftctl){
      shiftctl->sr_set_stv(false);
      gpio_clear_fast(_config.pin_ckv);
      EPD_DELAY_US(1);
      gpio_set_fast(_config.pin_ckv);
      shiftctl->sr_set_stv(true);
    } else {
      gpio_clear_fast(_config.pin_spv);
      gpio_clear_fast(_config.pin_ckv);
      EPD_DELAY_US(1);
      gpio_set_fast(_config.pin_ckv);
      gpio_set_fast(_config.pin_spv);
    }

  } else {
    if(shiftctl){
      shiftctl->sr_set_le(true);
      shiftctl->sr_set_le(false);
      gpio_clear_fast(_config.pin_ckv);
      EPD_DELAY_US(1);
      gpio_set_fast(_config.pin_ckv);
    } else {
      gpio_set_fast(_config.pin_le);
      gpio_clear_fast(_config.pin_le);
      gpio_clear_fast(_config.pin_ckv);
      EPD_DELAY_US(1);
      gpio_set_fast(_config.pin_ckv);
    }
  }

  // Reset ownership, flush AFIFO, and restart GDMA from the correct descriptor.
  // This prevents free-running DMA from preloading the next buffer into the AFIFO
  // before the CPU has written new row data there (which caused left-side artifacts).
  desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)desc);

  LCD_CAM.lcd_user.lcd_start = 1;
  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {}

    if(shiftctl){
      gpio_clear_fast(_config.pin_ckv);
      shiftctl->sr_set_le(true);
      shiftctl->sr_set_le(false);
      gpio_set_fast(_config.pin_ckv);
    } else {
      gpio_clear_fast(_config.pin_ckv);
      gpio_set_fast(_config.pin_le);
      EPD_DELAY_US(1);
      gpio_clear_fast(_config.pin_le);
      gpio_set_fast(_config.pin_ckv);
    }
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
  // I2C not used in ESP-IDF builds
#endif


  // ---- Configure EPD control pins ----
  // Pins managed by powerctl (PCA9555 or shift register) are set to -1.
  if (_config.pin_pwr >= 0) EPD_PIN_OUTPUT(_config.pin_pwr);
  if (_config.pin_spv >= 0) EPD_PIN_OUTPUT(_config.pin_spv);
  EPD_PIN_OUTPUT(_config.pin_ckv);
  EPD_PIN_OUTPUT(_config.pin_sph);
  if (_config.pin_oe  >= 0) EPD_PIN_OUTPUT(_config.pin_oe);
  if (_config.pin_le  >= 0) EPD_PIN_OUTPUT(_config.pin_le);
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

  // Cache the GDMA channel index for direct register access in sendRow().
  // LCD_CAM is peripheral 5 in the GDMA peri_sel register.
  _dma_channel_id = 0;
  for (int i = 0; i < 5; i++) {
    if (GDMA.channel[i].out.peri_sel.sel == 5) { _dma_channel_id = i; break; }
  }

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

  // ---- Set up DMA descriptors (one per buffer, stopped after each row) ----
  dma_desc2.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc2.dw0.suc_eof = 1;
  dma_desc2.dw0.size = packed_row_bytes;
  dma_desc2.dw0.length = packed_row_bytes;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2);
  dma_desc2.next = nullptr;

  dma_desc1.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc1.dw0.suc_eof = 1;
  dma_desc1.dw0.size = packed_row_bytes;
  dma_desc1.dw0.length = packed_row_bytes;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1);
  dma_desc1.next = nullptr;

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = (_config.width * _config.height) / 4;

  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  packed_paintbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  bitmask = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));

  // ── If a TPS chip is present, initialise the power controller ──
  if (_config.power.tps_addr != -1) {
    printf("\n── PowerCtl Init ──\n");
    powerctl = new epd_painter_powerctl();
    if (!powerctl->begin(_config)) {
      printf("FATAL: powerctl init failed!\n");
      while (1) EPD_DELAY_MS(1000);
    }
  }
  // ── If a shift register is present, initialise the shift controller ──
  if( _config.shift.data >= 0) {
    shiftctl = new epd_painter_powerctl_74HCT4094D();
    if (shiftctl != nullptr) {
      shiftctl->begin(_config);
    }
  }

  if (!(dma_buffer && packed_fastbuffer && packed_screenbuffer)) return false;

  _paint_active_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_active_sem); 
  _paint_buffer_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_buffer_sem); 

  xTaskCreatePinnedToCore(
    _paint_task_entry, "epd_paint", 8000, this, 10, &_paint_task_h, 0);


  // Clear previous screen, to keep DC balance.
  clear();

  if (_autoShutdown) {
    EPD_BootCtl boot(*this);
    if (boot.shutdownPending()) boot.shutdown();  // [[noreturn]] on shutdown path
  }

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
  if (!shiftctl) {
    EPD_PIN_LOW(_config.pin_spv);
  }
  EPD_PIN_LOW(_config.pin_sph);

  if (shiftctl) {
    shiftctl->powerOn();
  } else if (powerctl) {
    powerctl->powerOn();
  } else {
    EPD_PIN_HIGH(_config.pin_oe);
    EPD_DELAY_US(100);
    EPD_PIN_HIGH(_config.pin_pwr);
    EPD_DELAY_US(100);
  }

  if (!shiftctl) {
    gpio_clear_fast(_config.pin_spv);
  }
  gpio_clear_fast(_config.pin_ckv);
  EPD_DELAY_US(1);

  gpio_set_fast(_config.pin_ckv);
  if (!shiftctl) {
    gpio_set_fast(_config.pin_spv);
  }
}

void EPD_Painter::powerOff() {
  if (powerctl) {
    powerctl->powerOff();
  } else if (shiftctl) {
    shiftctl->powerOff();
  } else {
    EPD_PIN_LOW(_config.pin_oe);
    EPD_DELAY_US(100);
    EPD_PIN_LOW(_config.pin_pwr);
    EPD_DELAY_US(100);
  }
}

// Waveform tables are defined per-device in EPD_Painter_presets.h
// and stored in _config.waveforms.


// =============================================================================
// paint()
// =============================================================================
void EPD_Painter::paint(uint8_t *framebuffer) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);

  if (_config.rotation == Rotation::ROTATION_CW)
    compact_pixels_rotated_cw(framebuffer, packed_paintbuffer, _config.height, _config.width);
  else
    epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);


  paintStage=(interlace_mode?3:2);
  xSemaphoreGive(_paint_buffer_sem); 

  // wait until this buffer has been picked up by the paint loop.
  while(paintStage==(interlace_mode?3:2)){
      vTaskDelay(1);
  }
}

// =============================================================================
// paintPacked() — like paint() but skips compaction; buffer is already 2bpp
// =============================================================================
void EPD_Painter::paintPacked(const uint8_t* packed) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  memcpy(packed_paintbuffer, packed, (_config.width * _config.height) / 4);
  paintStage = (interlace_mode ? 3 : 2);
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == (interlace_mode ? 3 : 2)) {
    vTaskDelay(1);
  }
}

// =============================================================================
// unpaintPacked() — DC-balance pass: tells the driver the screen currently
// shows 'packed', then drives a blank (all-zero) frame so every pixel that
// was darkened gets a matching lightening pulse.
// =============================================================================
void EPD_Painter::unpaintPacked(const uint8_t* packed) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  memcpy(packed_screenbuffer, packed, packed_row_bytes * _config.height);
  memset(packed_paintbuffer,  0x00,  packed_row_bytes * _config.height);
  paintStage = (interlace_mode ? 3 : 2);
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == (interlace_mode ? 3 : 2)) {
    vTaskDelay(1);
  }
}

// =============================================================================
// paintLater
// =============================================================================
void EPD_Painter::paintLater(uint8_t *framebuffer) {
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    //const int64_t t0 = esp_timer_get_time();

    if (_config.rotation == Rotation::ROTATION_CW)
      compact_pixels_rotated_cw(framebuffer, packed_paintbuffer, _config.height, _config.width);
    else
      epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
    
   // printf("[rotate] compact_pixels_rotated_cw: %lld us\n", esp_timer_get_time() - t0);

    
    paintStage=interlace_mode?3:2;
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


      if (interlace_mode){
          bitmask[row] = epd_painter_ink(fb_row, sb_row, packed_row_bytes,  row%2?0xffffffff:0x00);
      } else {
          bitmask[row] = epd_painter_ink(fb_row, sb_row, packed_row_bytes,  0xffffffff);
      }
    }

    const uint8_t *lt_wf;
    const uint8_t *dk_wf;
    int wf_len;

    if (_config.quality == Quality::QUALITY_FAST) {
      lt_wf = &_config.waveforms.fast_lighter[0][0];
      dk_wf = &_config.waveforms.fast_darker[0][0];
      wf_len = 7;
    } else if(_config.quality == Quality::QUALITY_NORMAL) {
      lt_wf = &_config.waveforms.normal_lighter[0][0];
      dk_wf = &_config.waveforms.normal_darker[0][0];
      wf_len = 13;
    } else {
      lt_wf = &_config.waveforms.high_lighter[0][0];
      dk_wf = &_config.waveforms.high_darker[0][0];
      wf_len = 13;
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

     if (_config.quality == Quality::QUALITY_HIGH) {
        EPD_DELAY_MS(8);
      } else if (_config.quality == Quality::QUALITY_NORMAL) {
        EPD_DELAY_MS(2);
      }

    }

    vTaskDelay(1);  // yield once per frame: feeds WDT and lets application task run

    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    for (int row = 0; row < _config.height; ++row) {
      sendRow(row == 0, row == _config.height - 1);
    }

  }
}

// GPIO set/clear via direct register write — pins always <32
#define GPIO_SET(mask) (GPIO.out_w1ts = (mask))
#define GPIO_CLR(mask) (GPIO.out_w1tc = (mask))

// Clock one bit: set/clear data, then pulse clk
#define SHIFT_BIT(data_set, data_clr, clk_mask) \
  data_set; clk_mask; clk_mask

static void IRAM_ATTR shift_send(uint8_t reg,
                                  uint32_t data_mask,
                                  uint32_t clk_set,
                                  uint32_t clk_clr,
                                  uint32_t strobe_mask) {
  #define SEND_BIT(n) \
    if ((reg) & (1U << (n))) { GPIO_SET(data_mask); } else { GPIO_CLR(data_mask); } \
    GPIO_SET(clk_set); GPIO_CLR(clk_clr);

  SEND_BIT(7) SEND_BIT(6) SEND_BIT(5) SEND_BIT(4)
  SEND_BIT(3) SEND_BIT(2) SEND_BIT(1) SEND_BIT(0)
  #undef SEND_BIT

  GPIO_SET(strobe_mask);
  GPIO_CLR(strobe_mask);
}

void EPD_Painter::shiftOn(int bits) {
  shiftReg |= (uint8_t)bits;
  shift_send(shiftReg,
    1U << _config.shift.data,
    1U << _config.shift.clk,
    1U << _config.shift.clk,
    1U << _config.shift.strobe);
}

void EPD_Painter::shiftOff(int bits) {
  shiftReg &= ~(uint8_t)bits;
  shift_send(shiftReg,
    1U << _config.shift.data,
    1U << _config.shift.clk,
    1U << _config.shift.clk,
    1U << _config.shift.strobe);
}

// =============================================================================
// clearBuffers()
// Zero all packed pixel buffers so the DC-balance baseline is reset.
// Call this after painting a shutdown image and before cutting power, so that
// on the next boot unpaintPacked() works from a clean white baseline rather
// than the painted state that was left in PSRAM.
// =============================================================================
void EPD_Painter::clearBuffers() {
  const size_t packed_bytes = (size_t)_config.width * _config.height / 4;
  if (packed_fastbuffer)   memset(packed_fastbuffer,   0, packed_bytes);
  if (packed_screenbuffer) memset(packed_screenbuffer, 0, packed_bytes);
  if (packed_paintbuffer)  memset(packed_paintbuffer,  0, packed_bytes);
}

// =============================================================================
// clear()
// =============================================================================
void EPD_Painter::clear() {


  PanelPowerGuard guard(*this);

  const int packed_row_bytes = _config.width / 4;

  // First paint it white.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY); 
  memset(packed_paintbuffer, 0x00, packed_row_bytes * _config.height);
  paintStage=1;  /// only needs 1 pass.
  xSemaphoreGive(_paint_buffer_sem); 

  while (paintStage==1){
    vTaskDelay(1);  // Wait until paintloop starts up again
  }
  
  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  const uint8_t *lt_wf;
  int wf_len;

  // Send clear
  for (int phase = 0; phase < 4; phase++) {
    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;
    memset(dma_buffer1, pattern, packed_row_bytes);
    memset(dma_buffer2, pattern, packed_row_bytes);

    int totpass[] = { 6, 2, 4, 8}; //6 blacks, 2 whites, 4 blacks 8 whites.

    for (int passes = 0; passes < totpass[phase]; passes++) {
      for (int row = 0; row < _config.height; ++row) {
        sendRow(row == 0);
      }
      EPD_DELAY_MS(5);
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
// =============================================================================
// fxClear() — sweeping bar clear effect
//
// A thick bar travels left-to-right across the panel. Within the bar, each
// pixel is independently and randomly driven black or white on every pass,
// so adjacent pixels may be heading in opposite directions — the "fuzz".
// Left of the bar, pixels are settled white; right of the bar they are driven
// black and wait for the bar to sweep over them.
//
// Tuning constants (pixels, dimensionless):
//   BAR_W   — width of the active bar
//   STEP    — columns advanced per step
//   NPASSES — render passes per step (more = slower but better clearing)
// =============================================================================
void EPD_Painter::fxClear() {
  const int prb = _config.width / 4;   // packed row bytes

  // Wait for any in-progress paint to finish, then take exclusive control.
  // Unlike clear(), we do NOT issue a white frame first — the screenbuffer
  // must still reflect the current display state for the pixel mask to work.
  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  PanelPowerGuard guard(*this);

  const int H = _config.height;

  dma_buffer = dma_buffer1;

  // All white (0b00) screenbuffer pixels are driven fully black in the dark
  // zone and fully white in the light zone — no partial selection. This gives
  // maximum contrast: a solid black leading edge and a solid white trailing
  // edge. DC balance is maintained because dark and light zones are equal size.
  const int BAR_DARK  = 180;   // rows of black voltage (leading edge)
  const int BAR_LIGHT = 180;   // rows of white voltage (trailing edge)
  const int BAR_H     = BAR_DARK + BAR_LIGHT;
  const int STEP      = 15;    // 1 row per step → 60 dark + 60 light pulses per pixel

  for (int bar_top = -BAR_H; bar_top <= H; bar_top += STEP) {
    const int dark_top = bar_top + BAR_LIGHT;   // dark zone: lower (leading) half
    const int bar_bot  = bar_top + BAR_H;

    for (int row = 0; row < H; row++) {
      uint32_t* buf32 = reinterpret_cast<uint32_t*>(dma_buffer);

      if (row >= bar_top && row < dark_top) {
        // Light phase (upper/trailing): all white pixels → white voltage
        for (int i = 0; i < prb / 4; i++) {
          const uint32_t* sb32 = reinterpret_cast<const uint32_t*>(packed_screenbuffer + row * prb) + i;
          uint32_t either     = (*sb32 | (*sb32 >> 1)) & 0x55555555u;
          uint32_t pixel_mask = ~(either | (either << 1));
          buf32[i] = 0xAAAAAAAAu & pixel_mask;
        }
      } else if (row >= dark_top && row < bar_bot) {
        // Dark phase (lower/leading): all white pixels → black voltage
        for (int i = 0; i < prb / 4; i++) {
          const uint32_t* sb32 = reinterpret_cast<const uint32_t*>(packed_screenbuffer + row * prb) + i;
          uint32_t either     = (*sb32 | (*sb32 >> 1)) & 0x55555555u;
          uint32_t pixel_mask = ~(either | (either << 1));
          buf32[i] = 0x55555555u & pixel_mask;
        }
      } else {
        memset(dma_buffer, 0x00, prb);
      }
      sendRow(row == 0, false);
    }
  }


  // Final neutral flush — de-energise all pixels
  memset(dma_buffer1, 0x00, prb);
  memset(dma_buffer2, 0x00, prb);
  for (int row = 0; row < H; ++row) {
    sendRow(row == 0, row == H - 1);
  }

  // Screenbuffer is unchanged — non-white pixels were never driven, white pixels
  // were already 0x00. No update needed.

  xSemaphoreGive(_paint_active_sem);
}


// =============================================================================
// dither()
//
// Converts an 8bpp greyscale framebuffer (0=black … 255=white) in-place to
// the driver's 4-level encoding:  0=white  1=light-grey  2=dark-grey  3=black
//
// Algorithm: Floyd-Steinberg error diffusion.
// A single row of int16 scratch (next_err[width]) carries the below-row error
// forward so the main buffer never needs to be widened beyond uint8.
//
// Error distribution:
//          [x]  →  right:        7/16
//   below-left:  3/16   below:  5/16   below-right:  1/16
// =============================================================================
void EPD_Painter::dither(uint8_t* fb, uint16_t width, uint16_t height) {
    // Representative 8bpp values for each 2bpp level (0=white … 3=black)
    static const uint8_t kLevel8[4] = { 255, 170, 85, 0 };

    int16_t* next_err = (int16_t*)heap_caps_malloc(
        (size_t)width * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!next_err) return;
    memset(next_err, 0, (size_t)width * sizeof(int16_t));

    for (uint16_t y = 0; y < height; y++) {
        uint8_t* row = fb + (size_t)y * width;
        int16_t carry = 0;

        for (uint16_t x = 0; x < width; x++) {
            int32_t val = (int32_t)row[x] + carry + next_err[x];
            next_err[x] = 0;
            if (val < 0)   val = 0;
            if (val > 255) val = 255;

            // Quantise to nearest level (0=white … 3=black)
            uint8_t q;
            if      (val >= 213) q = 0;  // white
            else if (val >= 128) q = 1;  // light grey
            else if (val >= 43)  q = 2;  // dark grey
            else                 q = 3;  // black

            row[x] = q;

            int32_t err = val - (int32_t)kLevel8[q];

            carry = (err * 7) >> 4;
            if (y + 1 < height) {
                if (x > 0)         next_err[x - 1] += (int16_t)((err * 3) >> 4);
                                   next_err[x]     += (int16_t)((err * 5) >> 4);
                if (x + 1 < width) next_err[x + 1] += (int16_t)((err * 1) >> 4);
            }
        }
    }

    heap_caps_free(next_err);
}
