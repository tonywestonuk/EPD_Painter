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
#include "epd_pin_driver.h"

#ifdef ARDUINO
  #include "Wire.h"
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
#endif

// Define EPD_ASM_TIMING to enable assembly-function timing output.
// Prints compact_pixels, epd_painter_ink_dual, and convert_packed_fb_to_ink
// durations to serial each frame so you can compare old vs new .S builds.
//#define EPD_ASM_TIMING

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

// =============================================================================
// compact_pixels_180
//
// 180° rotation + 8bpp→2bpp compaction for a landscape (ROTATION_0) buffer.
// A 180° turn of a row-major image is just a full reversal of the pixel order
// (pixel (x,y) → (W-1-x, H-1-y) == linear index i → N-1-i).  So we read the
// source backwards in groups of 4 and pack forward, using the same MSB-first
// nibble layout as the linear pack — keeping panel coordinates correct.
//
// size must be a multiple of 4.
// =============================================================================
static void compact_pixels16(const uint8_t *src, uint8_t *dst, uint32_t n);

static IRAM_ATTR void compact_pixels_180(
    const uint8_t* src, uint8_t* dst, uint32_t size)
{
    const uint32_t nbytes = size / 4;
    const uint8_t* s = src + size - 1;        // last source pixel
    for (uint32_t b = 0; b < nbytes; b++) {
        dst[b] = ((s[ 0] & 3) << 6) | ((s[-1] & 3) << 4)
               | ((s[-2] & 3) << 2) |  (s[-3] & 3);
        s -= 4;
    }
}

extern "C" void epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint8_t *waveform, uint32_t chunk_flags);

extern "C" void epd_painter_convert_packed_fb_to_ink_or(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint8_t *waveform, uint32_t chunk_flags);

extern "C" uint32_t epd_painter_ink_dual(const uint8_t *packed_paintbuffer, uint8_t *packed_fastbuffer, uint8_t *packed_lightbuffer, uint8_t *packed_screenbuffer, uint32_t length, uint32_t *maskL_out);

static inline void epd_gpio_func_sel(int pin) {
  esp_rom_gpio_pad_select_gpio((gpio_num_t)pin);
}


#define PASS_COUNT 13


EPD_Painter::EPD_Painter(const Config &config, bool portrait) {
  _config = config;
  _preset = &config;
  if (portrait) _config.rotation = Rotation::ROTATION_CW;
}


void EPD_Painter::setQuality(Quality quality) {
  if (_grey16 && quality == Quality::QUALITY_FAST) {
    printf("[EPD_Painter] QUALITY_FAST cannot express 16 greys — call setGreyLevels(4) first\n");
    return;
  }
  _config.quality = quality;
}

// =============================================================================
// setGreyLevels() — 16-grey mode switch (decision engine phase C)
// =============================================================================
bool EPD_Painter::setGreyLevels(int levels) {
  if (levels != 4 && levels != 16) return false;
  const bool on = (levels == 16);
  if (on == _grey16) return true;

  if (_has_template) {
    printf("[EPD_Painter] setGreyLevels refused while a template is active"
           " — releaseTemplate() first\n");
    return false;
  }

  if (on && _config.quality == Quality::QUALITY_FAST) {
    printf("[EPD_Painter] 16-grey needs QUALITY_NORMAL or QUALITY_HIGH\n");
    return false;
  }

  if (on && !packed4_screenbuffer) {
    const size_t p4_bytes    = (size_t)_config.width * _config.height / 2;
    const size_t plane_bytes = (size_t)_config.width * _config.height / 4;
    packed4_paintbuffer  = (uint8_t *)heap_caps_aligned_alloc(16, p4_bytes, MALLOC_CAP_SPIRAM);
    packed4_screenbuffer = (uint8_t *)heap_caps_aligned_alloc(16, p4_bytes, MALLOC_CAP_SPIRAM);
    dec_spill = (uint8_t *)heap_caps_aligned_alloc(
        16, plane_bytes * (DEC_MAX_SWEEPS16 - 2), MALLOC_CAP_SPIRAM);
    dec_sweeps16 = (LineSweep *)heap_caps_aligned_alloc(
        4, sizeof(LineSweep) * _config.height * DEC_MAX_SWEEPS16, MALLOC_CAP_SPIRAM);
    if (!(packed4_paintbuffer && packed4_screenbuffer && dec_spill && dec_sweeps16)) {
      printf("[EPD_Painter] 16-grey buffer allocation failed\n");
      if (packed4_paintbuffer)  heap_caps_free(packed4_paintbuffer);
      if (packed4_screenbuffer) heap_caps_free(packed4_screenbuffer);
      if (dec_spill)            heap_caps_free(dec_spill);
      if (dec_sweeps16)         heap_caps_free(dec_sweeps16);
      packed4_paintbuffer = packed4_screenbuffer = dec_spill = nullptr;
      dec_sweeps16 = nullptr;
      return false;
    }
    _grey16_build_trains();
  }

  // Carry the physical screen state across the switch so ghost tracking
  // survives: 2bpp levels 0..3 <-> 4bpp levels {0, 5, 10, 15}.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  const size_t packed_bytes = (size_t)_config.width * _config.height / 4;
  if (on) {
    static const uint8_t lv[4] = { 0, 5, 10, 15 };
    for (size_t i = 0; i < packed_bytes; i++) {
      const uint8_t b = packed_screenbuffer[i];
      packed4_screenbuffer[i * 2]     = (uint8_t)((lv[(b >> 6) & 3] << 4) | lv[(b >> 4) & 3]);
      packed4_screenbuffer[i * 2 + 1] = (uint8_t)((lv[(b >> 2) & 3] << 4) | lv[b & 3]);
    }
  } else {
    for (size_t i = 0; i < packed_bytes; i++) {
      const uint8_t b0 = packed4_screenbuffer[i * 2];
      const uint8_t b1 = packed4_screenbuffer[i * 2 + 1];
      // nearest 4-level code: (v + 2) / 5 maps 0..15 -> 0..3
      packed_screenbuffer[i] = (uint8_t)(
          ((((b0 >> 4) & 15) + 2) / 5) << 6 | (((b0 & 15) + 2) / 5) << 4 |
          ((((b1 >> 4) & 15) + 2) / 5) << 2 | (((b1 & 15) + 2) / 5));
    }
  }
  _grey16 = on;
  _sb_guard_update();          // guard follows the active-mode buffer
  xSemaphoreGive(_paint_buffer_sem);
  return true;
}

// =============================================================================
// setDirectTransitions() — 4-level direct grey-to-grey engine
// (see DECISION_ENGINE.md "direct grey-to-grey transitions")
// =============================================================================
bool EPD_Painter::setDirectTransitions(bool on) {
  if (!on) { _decision_direct = false; return true; }
  if (!dec_spill_dir) {
    const size_t plane_bytes = (size_t)_config.width * _config.height / 4;
    dec_spill_dir = (uint8_t *)heap_caps_aligned_alloc(
        16, plane_bytes * (DEC_MAX_SWEEPS_DIR - 2), MALLOC_CAP_SPIRAM);
    dec_sweeps_dir = (LineSweep *)heap_caps_aligned_alloc(
        4, sizeof(LineSweep) * _config.height * DEC_MAX_SWEEPS_DIR,
        MALLOC_CAP_INTERNAL);
    if (!(dec_spill_dir && dec_sweeps_dir)) {
      printf("[EPD_Painter] direct-transition allocation failed\n");
      if (dec_spill_dir)  heap_caps_free(dec_spill_dir);
      if (dec_sweeps_dir) heap_caps_free(dec_sweeps_dir);
      dec_spill_dir = nullptr;
      dec_sweeps_dir = nullptr;
      return false;
    }
  }
  _decision_direct = true;
  return true;
}

// =============================================================================
// Template layer — a protected high-quality static layer under animation
// (see the header). The whole feature is one overlay: protected pixels are
// forced back to their template values in the packed paintbuffer before
// discovery, so no engine path can ever see them change.
// =============================================================================

bool EPD_Painter::setTemplate(const uint8_t *fb, Quality quality) {
  if (!fb) return false;
  if (_grey16 && quality == Quality::QUALITY_FAST) {
    printf("[EPD_Painter] template: FAST cannot express 16 greys\n");
    return false;
  }
  if (_has_template) releaseTemplate();

  // Paint the template at ITS quality, then restore the app's.
  const Quality qsave = _config.quality;
  _config.quality = quality;
  paint((uint8_t *)fb);
  while (!paintIdle()) EPD_DELAY_MS(2);
  _config.quality = qsave;

  // Capture the template by packing the CALLER'S fb — exactly as paint()
  // packs it, rotation included — never by copying the screenbuffer. The
  // screenbuffer is shared mutable state: another task's paintLater can
  // land between the template paint and a screenbuffer copy, baking its
  // dynamic content into the protected layer as un-erasable ghosts (seen
  // in the field: magitrac's tracker glyphs stamped into its GUI chrome).
  const size_t bytes =
      (size_t)_config.width * _config.height / (_grey16 ? 2 : 4);
  tpl_data = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  tpl_mask = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  if (!(tpl_data && tpl_mask)) {
    printf("[EPD_Painter] template allocation failed\n");
    if (tpl_data) heap_caps_free(tpl_data);
    if (tpl_mask) heap_caps_free(tpl_mask);
    tpl_data = tpl_mask = nullptr;
    return false;
  }
  if (_grey16)
    compact_pixels16((uint8_t *)fb, tpl_data,
                     (uint32_t)_config.width * _config.height);
  else if (_config.rotation == Rotation::ROTATION_CW)
    compact_pixels_rotated_cw((uint8_t *)fb, tpl_data,
                              _config.height, _config.width);
  else if (_config.rotation == Rotation::ROTATION_180)
    compact_pixels_180((uint8_t *)fb, tpl_data,
                       _config.width * _config.height);
  else
    epd_painter_compact_pixels((uint8_t *)fb, tpl_data,
                               _config.width * _config.height);
  for (size_t i = 0; i < bytes; i++) {
    const uint8_t b = tpl_data[i];
    uint8_t m = 0;
    if (_grey16) {
      if (b & 0xF0) m |= 0xF0;
      if (b & 0x0F) m |= 0x0F;
    } else {
      if (b & 0xC0) m |= 0xC0;
      if (b & 0x30) m |= 0x30;
      if (b & 0x0C) m |= 0x0C;
      if (b & 0x03) m |= 0x03;
    }
    tpl_mask[i] = m;
  }
  _tpl_grey16   = _grey16;
  _tpl_quality  = quality;
  _has_template = true;
  return true;
}

void EPD_Painter::releaseTemplate() {
  if (!_has_template) return;

  // Undo in the quality that painted: erase ONLY the template pixels,
  // with the template's trains. Everything else on screen is untouched.
  while (!paintIdle()) EPD_DELAY_MS(2);
  const Quality qsave = _config.quality;
  _config.quality = _tpl_quality;
  _has_template = false;              // stop the overlay for this paint

  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  const size_t bytes =
      (size_t)_config.width * _config.height / (_tpl_grey16 ? 2 : 4);
  uint8_t *pb = _tpl_grey16 ? packed4_paintbuffer : packed_paintbuffer;
  for (size_t i = 0; i < bytes; i++) pb[i] &= (uint8_t)~tpl_mask[i];
  paintStage = 1;                     // clear()'s submission pattern
  xSemaphoreGive(_paint_buffer_sem);
  while (!paintIdle()) EPD_DELAY_MS(2);

  _config.quality = qsave;
  heap_caps_free(tpl_data);
  heap_caps_free(tpl_mask);
  tpl_data = tpl_mask = nullptr;
}

// =============================================================================
// sendRow()
// =============================================================================
#ifdef EPD_ROW_PHASE_TIMING
// Per-paint accumulators: where does a row's time go? Reset at each
// drive's pass loop, printed after it.
int64_t g_rp_wait_us = 0, g_rp_pins_us = 0;
uint32_t g_rp_rows = 0;
#endif

void EPD_Painter::sendRow(bool firstLine, bool lastLine, bool noAdvance) {
#ifdef EPD_ROW_PHASE_TIMING
  const int64_t _rp_t0 = esp_timer_get_time();
#endif
  // Wait for LCD peripheral to finish consuming the previous row.
  // This also guarantees the previously-started DMA transfer is complete,
  // since the LCD FIFO cannot drain faster than DMA fills it.
  //long count=0;
  while (LCD_CAM.lcd_user.lcd_start) {}
#ifdef EPD_ROW_PHASE_TIMING
  g_rp_wait_us += esp_timer_get_time() - _rp_t0;
#endif
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

  // Reset the AFIFO while the bus is idle, then start GDMA *before* the
  // CKV/LE pin sequence below, so the DMA has time to fetch the descriptor
  // and prime the LCD AFIFO before lcd_start (esp-idf's i80 driver inserts
  // a delay here for the same reason).  The pin sequence takes ≥1us, so it
  // doubles as the prefill window at zero added row time.
  //
  // Note: the right-edge "image repeated ~16px apart" artifact once blamed
  // on this prefill race was actually the panel's source-driver shift chain
  // needing extra flush clocks — fixed by Config::row_pad_bytes (see
  // How_It_Works.md §7).  GDMA underflow flags confirmed the FIFO never
  // starved with this ordering.
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)desc);

#ifdef EPD_ROW_PHASE_TIMING
  const int64_t _rp_t2 = esp_timer_get_time();
#endif
  if (firstLine) {
    _pin_spv->set(false);
    _pin_ckv->set(false);
    EPD_DELAY_US(1);
    _pin_ckv->set(true);
    _pin_spv->set(true);
  } else if (noAdvance) {
    // Latch the previously transmitted data onto the source outputs without
    // advancing the gate: the current row drives this data for one more slot.
    _pin_le->set(true);
    _pin_le->set(false);
  } else {
    _pin_le->set(true);
    _pin_le->set(false);
    _pin_ckv->set(false);
    EPD_DELAY_US(1);
    _pin_ckv->set(true);
  }
#ifdef EPD_ROW_PHASE_TIMING
  g_rp_pins_us += esp_timer_get_time() - _rp_t2;
  g_rp_rows++;
#endif

  LCD_CAM.lcd_user.lcd_start = 1;
  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {}

    _pin_ckv->set(false);
    _pin_le->set(true);
    _pin_le->set(false);
    _pin_ckv->set(true);
  }
}

// =============================================================================
// begin()
// =============================================================================
bool EPD_Painter::begin() {

  #ifdef EPD_PAINTER_PRESET_AUTO
    if (!autoDetectBoard()) return false;
  #endif

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
  // Pins managed by powerctl (PCA9555) or encoded as EPD_SR_PIN() (shift register)
  // are skipped here — their hardware is initialised elsewhere.
  if (_config.pin_pwr >= 0 && !epd_pin_is_sr(_config.pin_pwr)) EPD_PIN_OUTPUT(_config.pin_pwr);
  if (_config.pin_spv >= 0 && !epd_pin_is_sr(_config.pin_spv)) EPD_PIN_OUTPUT(_config.pin_spv);
  EPD_PIN_OUTPUT(_config.pin_ckv);
  EPD_PIN_OUTPUT(_config.pin_sph);
  if (_config.pin_oe  >= 0 && !epd_pin_is_sr(_config.pin_oe))  EPD_PIN_OUTPUT(_config.pin_oe);
  if (_config.pin_le  >= 0 && !epd_pin_is_sr(_config.pin_le))  EPD_PIN_OUTPUT(_config.pin_le);
  EPD_PIN_OUTPUT(_config.pin_cl);


  packed_row_bytes = _config.width / 4;

  // Each row is sent with row_pad_bytes of trailing zeros: the panel's
  // source-driver shift chain has more stages than visible columns, so the
  // real pixel data needs extra CL clocks to reach the far (right-hand) end
  // before LE latches. Without the pad the last ~16 columns repeat content
  // from further left. Zero bytes are neutral drive — electrically harmless —
  // and cost ~50ns per row. See How_It_Works.md §7.
  const int dma_row_bytes = packed_row_bytes + _config.row_pad_bytes;

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
  LCD_CAM.lcd_user.lcd_dout_cyclelen = dma_row_bytes - 1;
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
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

  // Zero both buffers once so the pad region stays neutral forever —
  // pixel converts only ever write the first packed_row_bytes.
  memset(dma_buffer1, 0x00, dma_row_bytes);
  memset(dma_buffer2, 0x00, dma_row_bytes);

  dma_buffer = dma_buffer1;

  // ---- Set up DMA descriptors (one per buffer, stopped after each row) ----
  dma_desc2.dw0.suc_eof = 1;
  dma_desc2.dw0.size = dma_row_bytes;
  dma_desc2.dw0.length = dma_row_bytes;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2);
  dma_desc2.next = nullptr;

  dma_desc1.dw0.suc_eof = 1;
  dma_desc1.dw0.size = dma_row_bytes;
  dma_desc1.dw0.length = dma_row_bytes;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1);
  dma_desc1.next = nullptr;

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = (_config.width * _config.height) / 4;

  // Prefer internal RAM for speed, fall back to PSRAM if the contiguous
  // internal-heap block isn't available (e.g. after WiFi has been brought
  // up first — leaves the heap fragmented).  PSRAM-backed fastbuffer is
  // slower for the per-pixel ops in epd_painter_ink_dual() but boots cleanly.
  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));
  if (!packed_fastbuffer) {
    log_w("[EPD] packed_fastbuffer: internal alloc failed, retrying in PSRAM");
    packed_fastbuffer = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  }

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  packed_paintbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  // Light plane lives in PSRAM: ink writes and the pass loop reads only the
  // chunks flagged in bitmask_light, so access is sparse (boundary chunks),
  // not a full per-pass sweep.
  packed_lightbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  bitmask = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));
  bitmask_light = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));

  // Decision engine bookkeeping (small, internal): per-line todo words and
  // sweep lists — see DECISION_ENGINE.md.
  dec_todo = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));
  dec_sweeps = static_cast<LineSweep *>(
    heap_caps_aligned_alloc(4, _config.height * DEC_MAX_SWEEPS * sizeof(LineSweep),
                            MALLOC_CAP_INTERNAL));
  dec_nsweeps = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(4, _config.height, MALLOC_CAP_INTERNAL));
  memset(dec_train, 0, sizeof(dec_train));
  if (!(dec_todo && dec_sweeps && dec_nsweeps)) return false;

  // ── Create the power driver for this board ──
  if (_config.power.tps_addr != -1) {
    printf("\n── PowerCtl Init ──\n");
    auto* pc = new epd_painter_powerctl();
    if (!pc->begin(_config)) {
      printf("FATAL: powerctl init failed!\n");
      while (1) EPD_DELAY_MS(1000);
    }
    _powerDriver = pc;
  } else if (_config.shift.data >= 0) {
    if (_config.shift.driver == Shift::H716) {
      auto* h716 = new EPD_H716PowerDriver();
      h716->begin(_config.shift);
      _powerDriver = h716;
    } else {
      auto* h752 = new epd_painter_powerctl_74HCT4094D();
      h752->begin(_config);
      _powerDriver = h752;
    }
    _shiftReg = _powerDriver->isrController();
  } else {
    _powerDriver = new EPD_GpioPowerDriver(_config.pin_oe, _config.pin_pwr);
  }

  // ── Create per-pin drivers (GPIO or SR, determined by preset encoding) ──
  auto make_pin = [this](int16_t p) -> EPD_PinDriver* {
    if (epd_pin_is_sr(p)) return new EPD_SRPin(_shiftReg, epd_pin_sr_bit(p));
    return new EPD_GpioPin(uint8_t(p));
  };
  _pin_spv = make_pin(_config.pin_spv);
  _pin_ckv = make_pin(uint8_t(_config.pin_ckv));
  _pin_le  = make_pin(_config.pin_le);
  _pin_sph = make_pin(uint8_t(_config.pin_sph));

  if (!(dma_buffer && packed_fastbuffer && packed_screenbuffer)) return false;

  _paint_active_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_active_sem); 
  _paint_buffer_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_buffer_sem); 

  xTaskCreatePinnedToCore(
    _paint_task_entry, "epd_paint", 8000, this, 10, &_paint_task_h, 0);

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
// Transmit one row of the current dma_buffer through the source shift
// chain with NO latch and NO gate clock — data passes through and is
// never driven. Used to flush the chain at power-on.
void EPD_Painter::_pushRow() {
  uint32_t spin = 0;
  while (LCD_CAM.lcd_user.lcd_start && ++spin < 200000) {}
  if (LCD_CAM.lcd_user.lcd_start) return;
  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) { desc = &dma_desc1; dma_buffer = dma_buffer2; }
  else                           { desc = &dma_desc2; dma_buffer = dma_buffer1; }
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)desc);
  LCD_CAM.lcd_user.lcd_start = 1;
  spin = 0;
  while (LCD_CAM.lcd_user.lcd_start && ++spin < 200000) {}
}

void EPD_Painter::debugPinBench() {
  if (!_pin_le || !_pin_ckv) { printf("[bench] pins not initialised\n"); return; }
  const int64_t t0 = esp_timer_get_time();
  for (int i = 0; i < 1000; i++) { _pin_le->set(true); _pin_le->set(false); }
  const int64_t t1 = esp_timer_get_time();
  for (int i = 0; i < 1000; i++) { _pin_ckv->set(true); _pin_ckv->set(false); }
  const int64_t t2 = esp_timer_get_time();
  printf("[bench] LE pulse %.3f us  |  CKV pulse %.3f us  (1000 each)\n",
         (t1 - t0) / 1000.0, (t2 - t1) / 1000.0);
}

void EPD_Painter::powerOn() {
  printf("[EPD] panel power ON\n");
  _pin_le->set(false);
  _pin_spv->set(false);
  _pin_sph->set(false);

  _powerDriver->powerOn();

  // Flush the source shift chain. After a rail cycle the chain can hold
  // stale drive data at a variable offset — the first frame's rows then
  // land shifted a few pixels left/right (seen as thin ghost lines on a
  // square's edge, and magitrac's misaligned note ghosts). Two rows of
  // zeros pushed through with no latch leave it clean, aligned, and
  // full of float codes — exactly the state v1.1.0's per-paint closing
  // scan used to guarantee.
  if (dma_buffer) {
    // Rails need real settle time before the chain clocks reliably; the
    // pin driver only guarantees 100us.
    EPD_DELAY_MS(5);
    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    // Re-arm the source driver's line-start logic. SPH is held asserted
    // (low) for the panel's whole powered life, so its position state
    // machine is never re-synchronised after a rail ramp — the one thing
    // a data flush can't fix. Deassert for a full row of clocks to sweep
    // the start token out, then reassert so it re-enters at column 0.
    _pin_sph->set(true);   // deassert: no start token while we clock
    _pushRow();
    _pin_sph->set(false);  // reassert: token re-enters at column 0
    _pushRow();
    _pushRow();
  }

  _pin_spv->set(false);
  _pin_ckv->set(false);
  EPD_DELAY_US(1);
  _pin_ckv->set(true);
  _pin_spv->set(true);
}

void EPD_Painter::powerOff() {
  printf("[EPD] panel power OFF\n");
  _powerDriver->powerOff();
}


int EPD_Painter::readPanelTemperatureC() {
  if (!_powerDriver) return EPD_PowerDriver::TEMP_UNAVAILABLE;
  return _powerDriver->readTemperatureC();
}

// Waveform tables are defined per-device in EPD_Painter_presets.h
// and stored in _config.waveforms.


// =============================================================================
// debugRowTest() — TEMPORARY hardware probe for the double-latch question.
//
// Drives a stripe pattern (1 black row every 6) three ways:
//   band 1 rows   0..179: stripe row = one BLACK transmission (reference)
//   band 2 rows 180..359: stripe row = BLACK transmission, then a FLOAT
//                         transmission latched with noAdvance
//   band 3 rows 360..539: stripe row = FLOAT transmission, then a BLACK
//                         transmission latched with noAdvance
//
// If a gate row can be driven twice: all three bands show identical stripe
// alignment. If the second latch lands on a different row, band 3's stripes
// are displaced relative to bands 1/2 — and the displacement direction gives
// the true latch/advance pairing.
//
// Raw drive codes per pixel pair: 0b10 = darken, 0b01 = whiten, 0b00 = float.
// =============================================================================
void EPD_Painter::debugRowTest() {
  PanelPowerGuard guard(*this);
  const int H = _config.height;
  const uint8_t BLACK = 0x55;   // all 4 pixels: drive dark (code 0b01 — the
                                // darker waveform tables are built from 1s)
  const uint8_t FLOAT = 0x00;

  for (int pass = 0; pass < 8; pass++) {
    bool no_adv = false;
    for (int row = 0; row < H; row++) {
      const bool stripe = (row % 6) == 0;
      const int  band   = row < 180 ? 1 : (row < 360 ? 2 : 3);

      uint8_t first  = FLOAT;
      uint8_t second = FLOAT;
      bool    dbl    = false;
      if (stripe) {
        if (band == 1)      { first = BLACK; }
        else if (band == 2) { first = BLACK; second = FLOAT; dbl = true; }
        else                { first = FLOAT; second = BLACK; dbl = true; }
      }

      memset(dma_buffer, first, packed_row_bytes);
      sendRow(row == 0, false, no_adv);
      no_adv = false;

      if (dbl) {
        memset(dma_buffer, second, packed_row_bytes);
        sendRow(false, false, false);
        no_adv = true;
      }
    }
    if (no_adv) {
      memset(dma_buffer, FLOAT, packed_row_bytes);
      sendRow(false, false, true);
    }
  }

  // Final all-float pass to leave the panel undriven
  for (int row = 0; row < H; row++) {
    memset(dma_buffer, FLOAT, packed_row_bytes);
    sendRow(row == 0, row == H - 1);
  }
}

// =============================================================================
// paint()
// =============================================================================
// 16-grey pack: 8bpp level codes (0..15) → 4bpp, first pixel in the high
// nibble. Phase C supports ROTATION_0 only.
static void compact_pixels16(const uint8_t *src, uint8_t *dst, uint32_t n) {
  for (uint32_t i = 0; i < n; i += 2)
    *dst++ = (uint8_t)(((src[i] & 15) << 4) | (src[i + 1] & 15));
}

void EPD_Painter::paint(uint8_t *framebuffer) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);

#ifdef EPD_ASM_TIMING
  const int64_t _cp_t0 = esp_timer_get_time();
#endif
  if (_grey16) {
    if (_config.rotation != Rotation::ROTATION_0) {
      static bool warned = false;
      if (!warned) { warned = true; printf("[EPD_Painter] 16-grey paint(): only ROTATION_0 is supported (phase C)\n"); }
    }
    compact_pixels16(framebuffer, packed4_paintbuffer, (uint32_t)_config.width * _config.height);
  }
  else if (_config.rotation == Rotation::ROTATION_CW)
    compact_pixels_rotated_cw(framebuffer, packed_paintbuffer, _config.height, _config.width);
  else if (_config.rotation == Rotation::ROTATION_180)
    compact_pixels_180(framebuffer, packed_paintbuffer, _config.width * _config.height);
  else
    epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
#ifdef EPD_ASM_TIMING
  printf("[paint] compact_pixels: %lld us\n", esp_timer_get_time() - _cp_t0);
#endif

  paintStage=2;
  xSemaphoreGive(_paint_buffer_sem); 

  // wait until this buffer has been picked up by the paint loop.
  while(paintStage==2){
      vTaskDelay(1);
  }
}

// =============================================================================
// paintPacked() — like paint() but skips compaction; buffer is already 2bpp.
// line_repeat > 1: `packed` holds height/line_repeat rows, each driven
// line_repeat times (vertical pixel doubling for reduced-bandwidth streams).
// =============================================================================
void EPD_Painter::paintPacked(const uint8_t* packed, int line_repeat) {
  if (_grey16) {
    printf("[EPD_Painter] paintPacked() is a 4-level path — call setGreyLevels(4) first\n");
    return;
  }
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  if (line_repeat <= 1) {
    memcpy(packed_paintbuffer, packed, (_config.width * _config.height) / 4);
  } else {
    uint8_t* dst = packed_paintbuffer;
    const int src_rows = _config.height / line_repeat;
    for (int r = 0; r < src_rows; ++r) {
      for (int k = 0; k < line_repeat; ++k) {
        memcpy(dst, packed + r * packed_row_bytes, packed_row_bytes);
        dst += packed_row_bytes;
      }
    }
  }
  paintStage = 2;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 2) {
    vTaskDelay(1);
  }
}

// =============================================================================
// unpaintPacked() — DC-balance pass: tells the driver the screen currently
// shows 'packed', then drives a blank (all-zero) frame so every pixel that
// was darkened gets a matching lightening pulse.
// =============================================================================
void EPD_Painter::unpaintPacked(const uint8_t* packed) {
  if (_grey16) {
    printf("[EPD_Painter] unpaintPacked() is a 4-level path — call setGreyLevels(4) first\n");
    return;
  }
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  memcpy(packed_screenbuffer, packed, packed_row_bytes * _config.height);
  memset(packed_paintbuffer,  0x00,  packed_row_bytes * _config.height);
  paintStage = 2;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 2) {
    vTaskDelay(1);
  }
}

// =============================================================================
// paintLater
// =============================================================================
void EPD_Painter::paintLater(uint8_t *framebuffer) {
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
#ifdef EPD_ASM_TIMING
    const int64_t _cpl_t0 = esp_timer_get_time();
#endif
    if (_grey16)
      compact_pixels16(framebuffer, packed4_paintbuffer, (uint32_t)_config.width * _config.height);
    else if (_config.rotation == Rotation::ROTATION_CW)
      compact_pixels_rotated_cw(framebuffer, packed_paintbuffer, _config.height, _config.width);
    else if (_config.rotation == Rotation::ROTATION_180)
      compact_pixels_180(framebuffer, packed_paintbuffer, _config.width * _config.height);
    else
      epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
#ifdef EPD_ASM_TIMING
    printf("[paintLater] compact_pixels: %lld us\n", esp_timer_get_time() - _cpl_t0);
#endif

    
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

    // Dual-plane delta pass: reads the new frame straight out of the (PSRAM)
    // paintbuffer and emits both drive directions — dark plane into the
    // internal fastbuffer, light plane into PSRAM — with a chunk mask each.
    // There is no direction priority and no deferral: chunks flagged in both
    // masks are resolved in the pass loop by driving their row twice.
    // The buffer semaphore is held for the whole sweep because the
    // paintbuffer is being read directly; paint()/paintPacked() block on it
    // rather than on paintStage if they arrive mid-sweep.
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    _sb_guard_check();
    // Template layer: force protected pixels back to their template
    // values before discovery — every engine path (SIMD, C, direct,
    // 16-grey) then simply sees them as unchanged.
    // Tripwire: an app that never calls setTemplate() must never see
    // this fire — if it does, something scribbled on the driver object
    // (heap overflow elsewhere) and flipped the flag with garbage
    // pointers. Announce loudly instead of stamping junk on the glass.
    if (_has_template && (!tpl_data || !tpl_mask)) {
      _tpl_trip_hits++;
      printf("[EPD_Painter] CORRUPTION: template flag set with null "
             "planes (tpl_data=%p tpl_mask=%p) — disabling overlay\n",
             tpl_data, tpl_mask);
      _has_template = false;
    }
    if (_has_template && _tpl_grey16 == _grey16) {
      static bool announced = false;
      if (!announced) {
        announced = true;
        _tpl_trip_hits++;
        printf("[EPD_Painter] template overlay ACTIVE (data=%p mask=%p)\n",
               tpl_data, tpl_mask);
      }
      const size_t words =
          (size_t)_config.width * _config.height / (_grey16 ? 2 : 4) / 4;
      uint32_t *p = (uint32_t *)(_grey16 ? packed4_paintbuffer
                                         : packed_paintbuffer);
      const uint32_t *t = (const uint32_t *)tpl_data;
      const uint32_t *m = (const uint32_t *)tpl_mask;
      for (size_t i = 0; i < words; i++) p[i] = (p[i] & ~m[i]) | t[i];
    }

#ifdef EPD_ASM_TIMING
    const int64_t _ink_t0 = esp_timer_get_time();
#endif
    uint32_t any_work = 0;
    const LineSweep *sweep_list = dec_sweeps;
    int sweep_stride = DEC_MAX_SWEEPS;
    if (_grey16) {
      // 16-grey discovery batches decisions into sweep lists directly —
      // up to 10 sweeps per line, greedy first-appearance order.
      any_work = _decision_discover16();
      sweep_list = dec_sweeps16;
      sweep_stride = DEC_MAX_SWEEPS16;
    } else if (_decision_direct) {
      // Direct-transition engine: greedy-sweep discovery over the 2bpp
      // buffers, grey-to-grey pixels driven in one paint where a pair
      // train is loaded (see DECISION_ENGINE.md).
      any_work = _decision_discover_direct();
      sweep_list = dec_sweeps_dir;
      sweep_stride = DEC_MAX_SWEEPS_DIR;
    } else if (_decision_engine) {
      // Decision engine: C discovery — planes, masks, per-line todo words.
      any_work = _decision_discover();
      _decision_batch_compat();
    } else {
      for (int row = 0; row < _config.height; row++) {
        const uint8_t *pb_row = packed_paintbuffer + row * packed_row_bytes;
        uint8_t *fbD_row = packed_fastbuffer  + row * packed_row_bytes;
        uint8_t *fbL_row = packed_lightbuffer + row * packed_row_bytes;
        uint8_t *sb_row  = packed_screenbuffer + row * packed_row_bytes;

        bitmask[row] = epd_painter_ink_dual(pb_row, fbD_row, fbL_row, sb_row,
                                            packed_row_bytes, &bitmask_light[row]);
        any_work |= bitmask[row] | bitmask_light[row];
      }
      // Both 4-level engines feed the same per-line sweep lists (the fixed
      // compatibility mapping — the July dual-plane layout, exactly).
      _decision_batch_compat();
    }
#ifdef EPD_ASM_TIMING
    printf("[paint_task] ink discovery (all rows): %lld us\n", esp_timer_get_time() - _ink_t0);
#endif
    xSemaphoreGive(_paint_buffer_sem);

    paintStage-=1;

    if (!any_work) {
      // Nothing changed anywhere — skip the drive passes entirely. Under
      // continuous streaming this makes the mop-up second cycle nearly free;
      // for stills it makes repainting an unchanged frame a no-op.
      vTaskDelay(1);
      continue;
    }

    PanelPowerGuard guard(*this);

    const uint8_t *lt_wf;
    const uint8_t *dk_wf;
    int wf_len;

    const Waveforms &wf = _config.waveforms;

    if (_config.quality == Quality::QUALITY_FAST) {
      lt_wf = &wf.fast_lighter[0][0];
      dk_wf = &wf.fast_darker[0][0];
      wf_len = 7;
    } else if(_config.quality == Quality::QUALITY_NORMAL) {
      lt_wf = &wf.normal_lighter[0][0];
      dk_wf = &wf.normal_darker[0][0];
      wf_len = 13;
    } else {
      lt_wf = &wf.high_lighter[0][0];
      dk_wf = &wf.high_darker[0][0];
      wf_len = 13;
    }

#ifdef EPD_ASM_TIMING
    int64_t _conv_total = 0;
    int _dbl_rows = 0;
#endif
    // Decision -> train bindings for this paint. 16-grey mode binds the
    // formula train library (30 trains, ids 2..31; placeholder until phase
    // D calibrates them). 4-level mode binds the calibrated tables —
    // apply(g) is the darker table's row g-1, remove(g) the lighter table's
    // (id = (level << 1) | dir) — and clears the higher ids so a stale
    // 16-grey binding can never leak into a 4-level paint.
    if (_grey16) {
      for (int id = 2; id < DEC_IDS; id++) dec_train[id] = dec_trains16[id];
    } else {
      for (int g = 1; g <= 3; g++) {
        dec_train[(g << 1) | 0] = dk_wf + (g - 1) * wf_len;
        dec_train[(g << 1) | 1] = lt_wf + (g - 1) * wf_len;
      }
      for (int id = 8; id < DEC_IDS; id++) dec_train[id] = nullptr;
      if (_decision_direct) {
        // Direct grey-to-grey trains: loaded pairs only — discovery never
        // emits an unloaded pair's id, but keep binding and gate in step.
        for (int f = 1; f <= 3; f++)
          for (int t = 1; t <= 3; t++)
            if (f != t && (_dir_loaded & (1u << ((f << 2) | t))))
              dec_train[directId(f, t)] = dec_trains_dir[(f << 2) | t];
      }
    }

    // Pass count = length of the longest train in play (DECISION_ENGINE.md).
    // Passes beyond every active train's last non-float code drive nothing;
    // skipping them costs no dose fidelity because each executed pass keeps
    // its exact constant period, and every pixel's final drive still gets a
    // full-period window before the next latch. dec_todo holds the frame's
    // decision ids from discovery. 4-level mode keeps its calibrated fixed
    // pass count.
    // Per-id train lengths: base trains are exactly wf_len codes (the
    // FAST tables are 7-long rows — reading past them would overrun into
    // the next level's row), direct trains up to DEC_WF_LEN_DIR. The
    // bcast loop floats any id past its own length.
    uint8_t dec_len[DEC_IDS];
    for (int id = 0; id < DEC_IDS; id++) dec_len[id] = (uint8_t)wf_len;

    if (_grey16) {
      uint32_t inplay = 0;
      for (int row = 0; row < _config.height; row++) inplay |= dec_todo[row];

      // Temporal partition (DECISION_ENGINE.md "direct grey-to-grey"):
      // a grey-to-grey pixel took BOTH a remove and an apply this frame.
      // Its remove fully erases first, then its apply drives from erased
      // glass — so each apply id shifts right by the longest remove
      // among ITS OWN from-partners this frame (a deep erase elsewhere
      // never stalls a shallow transition: 15->3 costs 13+3 passes
      // without dragging 3->15 to 26). Disjoint pass ranges keep the
      // pixel's two sweeps OR-safe; DC composes -Q(from) + Q(to) from
      // the tuned tables. Frames without grey-to-grey shift nothing.
      for (int to = 1; to <= 15; to++) {
        const uint16_t fromset = dec_gg_from[to];
        if (!fromset) continue;
        const int aid = to << 1;
        if (!(inplay & (1u << aid)) || !dec_train[aid]) continue;
        int R = 0;
        for (int f = 1; f <= 15; f++) {
          if (!(fromset & (1u << f))) continue;
          const uint8_t *rt = dec_train[(f << 1) | 1];
          if (!rt) continue;
          for (int p = R; p < DEC_WF_LEN16; p++)
            if (rt[p]) R = p + 1;
        }
        if (R > 0) {
          uint8_t *sh = dec_shifted[aid];
          memset(sh, 0, R);
          memcpy(sh + R, dec_train[aid], DEC_WF_LEN16);
          dec_train[aid] = sh;
          dec_len[aid] = (uint8_t)(R + DEC_WF_LEN16);
        }
      }

      int need = 1;
      for (int id = 2; id < DEC_IDS; id++) {
        if (!(inplay & (1u << id)) || !dec_train[id]) continue;
        const int len = dec_len[id];
        for (int p = 0; p < len; p++)
          if (dec_train[id][p] && p >= need) need = p + 1;
      }
      wf_len = need;
    } else if (_decision_direct) {
      // Direct trains may be LONGER than the quality's pass count (deep
      // lightening transitions need erase + re-darken room). Extend the
      // frame to the longest direct train actually in play — content
      // without deep transitions pays nothing (DECISION_ENGINE.md).
      uint32_t inplay = 0;
      for (int row = 0; row < _config.height; row++) inplay |= dec_todo[row];
      for (int id = 16; id < DEC_IDS; id++) {
        if (!(inplay & (1u << id)) || !dec_train[id]) continue;
        dec_len[id] = DEC_WF_LEN_DIR;
        for (int p = wf_len; p < DEC_WF_LEN_DIR; p++)
          if (dec_train[id][p]) wf_len = p + 1;
      }
    }

    // The pass loop consumes per-line sweep lists: a sweep = one staged
    // 2bpp slot plane + a chunk mask + 3 decision ids naming the waveform
    // train each slot indexes. The engine ORs a line's sweeps into the row
    // accumulator before the single latch; the k-way OR is safe because a
    // pixel belongs to exactly one sweep, so the others write float for it
    // (see DECISION_ENGINE.md). Phase B lists are the fixed dual-plane
    // pair: apply-decisions in one sweep, remove-decisions in the other,
    // pixel-disjoint by construction.
#ifdef EPD_GREY16_PASS_TIMING
    int64_t _g16_rowloop_min = 0, _g16_rowloop_max = 0;
#endif
#ifdef EPD_ROW_PHASE_TIMING
    { extern int64_t g_rp_wait_us, g_rp_pins_us; extern uint32_t g_rp_rows;
      g_rp_wait_us = g_rp_pins_us = 0; g_rp_rows = 0; }
#endif
    for (uint8_t pass = 0; pass < wf_len; pass++) {
      const int64_t pass_t0 = esp_timer_get_time();
      // Per-decision broadcast bytes for this pass: code * 0x55 replicates
      // a 2-bit drive code across the 4 pixels of a byte. Ids 0/1 (level
      // 0 = white) have no trains and always float.
      uint8_t bcast[DEC_IDS];
      bcast[0] = bcast[1] = 0;
      for (int id = 2; id < DEC_IDS; id++)
        bcast[id] = (dec_train[id] && pass < dec_len[id])
                        ? (uint8_t)(dec_train[id][pass] * 0x55) : 0;

      for (int row = 0; row < _config.height; row++) {
#ifdef EPD_ASM_TIMING
        const int64_t _conv_t0 = esp_timer_get_time();
        if (pass == 0 && (bitmask[row] & bitmask_light[row])) _dbl_rows++;
#endif
        // Chunks with no work in any sweep must be explicitly floated
        // (zeroed) — convert skips them and would otherwise leave the
        // previous row's drive data. The first sweep overwrites its active
        // chunks, later sweeps OR on top: all of a line's ink rides in ONE
        // transmission with full retention dose. (Driving the row once per
        // sweep instead does NOT work — a second latch cuts the first
        // data's retention from a full pass period to one ~30us row slot,
        // starving its dose to nothing. Verified optically with the
        // debugRowTest() stripe probe.)
        memset(dma_buffer, 0x00, packed_row_bytes);
        const LineSweep *ls = &sweep_list[row * sweep_stride];
        const int n = dec_nsweeps[row];
        for (int s = 0; s < n; s++) {
          // Table entry order is slot 3, 2, 1 (slot 0 = float).
          uint8_t tbl[3] = { bcast[ls[s].dec[2]], bcast[ls[s].dec[1]], bcast[ls[s].dec[0]] };
          if (s == 0)
            epd_painter_convert_packed_fb_to_ink(ls[s].plane_row, dma_buffer, packed_row_bytes, tbl, ls[s].mask);
          else
            epd_painter_convert_packed_fb_to_ink_or(ls[s].plane_row, dma_buffer, packed_row_bytes, tbl, ls[s].mask);
        }
#ifdef EPD_ASM_TIMING
        _conv_total += esp_timer_get_time() - _conv_t0;
#endif
        sendRow(row == 0, false, false);
      }

     if (_grey16) {
        // Constant pass period: dose = time between a row's latches = the
        // pass duration, and the row loop's length varies with content
        // (sweep count). Pad to a fixed period so dose depends only on
        // the trains — but never squeeze the inter-pass settle below the
        // quality's floor. Trains are calibrated at this period.
        const int64_t elapsed = esp_timer_get_time() - pass_t0;
#ifdef EPD_GREY16_PASS_TIMING
        if (pass == 0 || elapsed < _g16_rowloop_min) _g16_rowloop_min = elapsed;
        if (elapsed > _g16_rowloop_max) _g16_rowloop_max = elapsed;
#endif
        const int64_t period = (_config.quality == Quality::QUALITY_HIGH)
                                   ? _config.g16_pass_us_high
                                   : _config.g16_pass_us_normal;
        const int64_t settle_floor =
            (_config.quality == Quality::QUALITY_HIGH) ? 8000 : 4000;
        int64_t pad = period - elapsed;
        if (pad < settle_floor) pad = settle_floor;   // overrun: keep settle
        EPD_DELAY_MS((uint32_t)((pad + 500) / 1000));
      } else if (_config.quality == Quality::QUALITY_HIGH) {
        EPD_DELAY_MS(8);
      } else if (_config.quality == Quality::QUALITY_NORMAL) {
        EPD_DELAY_MS(4);
      }
      // QUALITY_FAST = No delay.
    }
#ifdef EPD_GREY16_PASS_TIMING
    if (_grey16)
      printf("[grey16] pass row-loop %lld..%lld us (period %d us)\n",
             (long long)_g16_rowloop_min, (long long)_g16_rowloop_max,
             (int)(_config.quality == Quality::QUALITY_HIGH
                       ? _config.g16_pass_us_high : _config.g16_pass_us_normal));
#endif
#ifdef EPD_ASM_TIMING
    printf("[paint_task] convert_packed_fb_to_ink (all passes, all rows, darker+lighter): %lld us\n", _conv_total);
    printf("[paint_task] merged (mixed-chunk) rows: %d of %d\n", _dbl_rows, _config.height);
#endif
#ifdef EPD_ROW_PHASE_TIMING
    { extern int64_t g_rp_wait_us, g_rp_pins_us; extern uint32_t g_rp_rows;
      if (g_rp_rows)
        printf("[rowprof] rows=%u  dma-wait=%lld us (%.1f/row)  pins=%lld us (%.1f/row)\n",
               (unsigned)g_rp_rows, (long long)g_rp_wait_us,
               (double)g_rp_wait_us / g_rp_rows,
               (long long)g_rp_pins_us, (double)g_rp_pins_us / g_rp_rows); }
#endif

    vTaskDelay(1);  // yield once per frame: feeds WDT and lets application task run

    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    for (int row = 0; row < _config.height; ++row) {
      sendRow(row == 0, row == _config.height - 1);
    }

    _sb_guard_update();
    _paints_done.fetch_add(1);
  }
}

// GPIO set/clear via direct register write — pins always <32
#define GPIO_SET(mask) (GPIO.out_w1ts = (mask))
#define GPIO_CLR(mask) (GPIO.out_w1tc = (mask))

// Clock one bit: set/clear data, then pulse clk
#define SHIFT_BIT(data_set, data_clr, clk_mask) \
  data_set; clk_mask; clk_mask

// =============================================================================
// clearBuffers()
// Zero all packed pixel buffers so the DC-balance baseline is reset.
// Call this after painting a shutdown image and before cutting power, so that
// on the next boot unpaintPacked() works from a clean white baseline rather
// than the painted state that was left in PSRAM.
// =============================================================================
// ---- screenbuffer state guard (see header) --------------------------------
// Full shadow copy: on mismatch we fingerprint the foreign write — where
// it landed, how much, and what the bytes look like (ASCII = a string
// overflow, structured data = someone's buffer).

void EPD_Painter::_sb_guard_update() {
  if (!_sb_guard_on) return;
  const uint8_t *sb = _grey16 ? packed4_screenbuffer : packed_screenbuffer;
  if (!sb) return;
  const size_t bytes =
      (size_t)_config.width * _config.height / (_grey16 ? 2 : 4);
  if (!_sb_shadow)
    _sb_shadow = (uint8_t *)heap_caps_malloc(
        (size_t)_config.width * _config.height / 2, MALLOC_CAP_SPIRAM);
  if (!_sb_shadow) { _sb_guard_on = false; return; }
  memcpy(_sb_shadow, sb, bytes);
  _sb_guard_valid = true;
}

void EPD_Painter::_sb_guard_check() {
  if (!_sb_guard_on || !_sb_guard_valid || !_sb_shadow) return;
  const uint8_t *sb = _grey16 ? packed4_screenbuffer : packed_screenbuffer;
  if (!sb) return;
  const size_t bytes =
      (size_t)_config.width * _config.height / (_grey16 ? 2 : 4);
  if (memcmp(_sb_shadow, sb, bytes) == 0) return;

  _sb_guard_hits++;
  size_t first = 0, last = 0, count = 0;
  for (size_t i = 0; i < bytes; i++) {
    if (_sb_shadow[i] != sb[i]) {
      if (!count) first = i;
      last = i;
      count++;
    }
  }
  printf("[EPD_Painter] CORRUPTION #%lu: foreign write into screenbuffer "
         "(%p): offsets %u..%u, %u bytes changed. int free=%u largest=%u\n",
         (unsigned long)_sb_guard_hits, sb, (unsigned)first, (unsigned)last,
         (unsigned)count,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  const size_t dump = (bytes - first < 32) ? bytes - first : 32;
  printf("[EPD_Painter]   was:");
  for (size_t i = 0; i < dump; i++) printf(" %02x", _sb_shadow[first + i]);
  printf("\n[EPD_Painter]   now:");
  for (size_t i = 0; i < dump; i++) printf(" %02x", sb[first + i]);
  printf("\n[EPD_Painter]   txt: ");
  for (size_t i = 0; i < dump; i++) {
    const uint8_t c = sb[first + i];
    printf("%c", (c >= 32 && c < 127) ? c : '.');
  }
  printf("\n");
}

void EPD_Painter::clearBuffers() {
  const size_t packed_bytes = (size_t)_config.width * _config.height / 4;
  if (packed_fastbuffer)   memset(packed_fastbuffer,   0, packed_bytes);
  if (packed_screenbuffer) memset(packed_screenbuffer, 0, packed_bytes);
  if (packed_paintbuffer)  memset(packed_paintbuffer,  0, packed_bytes);
  if (packed4_screenbuffer) memset(packed4_screenbuffer, 0, packed_bytes * 2);
  if (packed4_paintbuffer)  memset(packed4_paintbuffer,  0, packed_bytes * 2);
  _sb_guard_update();
}

// =============================================================================
// computeDirtyRects()
// Scans packed_screenbuffer vs packed_paintbuffer row by row.  Each row is
// checked byte-by-byte (1 byte = 4 pixels at 2bpp) to find the leftmost and
// rightmost changed byte.
//
// Rows are grouped into rectangles.  When a clean row is encountered inside an
// open rectangle the "wasted" pixel count grows by (rect_width × 4).  Once
// wasted exceeds tolerance the rectangle is closed at the last dirty row and a
// new one will be opened at the next dirty row.
//
// tolerance = 0  → only consecutive dirty rows are merged (tight rects)
// tolerance = large → everything collapses toward a single full-screen rect
// =============================================================================
int EPD_Painter::computeDirtyRects(Rect* out_rects, int max_rects, int tolerance) const {
    if (!packed_screenbuffer || !packed_paintbuffer || max_rects <= 0) return 0;

    const int prb = _config.width / 4;
    const int H   = _config.height;

    int  count   = 0;
    bool open    = false;
    int  rect_y0 = 0, rect_y1 = 0;
    int  rect_bx0 = 0, rect_bx1 = 0;
    int  wasted  = 0;

    for (int row = 0; row < H && count < max_rects; row++) {
        const uint8_t* s = packed_screenbuffer + row * prb;
        const uint8_t* p = packed_paintbuffer  + row * prb;

        // Find changed byte span for this row
        int bx0 = prb, bx1 = 0;
        for (int bx = 0; bx < prb; bx++) {
            if (s[bx] != p[bx]) {
                if (bx < bx0) bx0 = bx;
                bx1 = bx + 1;
            }
        }
        const bool dirty = (bx0 < bx1);

        if (dirty) {
            if (!open) {
                open     = true;
                rect_y0  = row;
                rect_y1  = row + 1;
                rect_bx0 = bx0;
                rect_bx1 = bx1;
                wasted   = 0;
            } else {
                if (bx0 < rect_bx0) rect_bx0 = bx0;
                if (bx1 > rect_bx1) rect_bx1 = bx1;
                rect_y1 = row + 1;
                wasted  = 0;
            }
        } else if (open) {
            wasted += (rect_bx1 - rect_bx0) * 4;
            if (wasted > tolerance) {
                out_rects[count++] = {
                    (int16_t)(rect_bx0 * 4),
                    (int16_t) rect_y0,
                    (int16_t)((rect_bx1 - rect_bx0) * 4),
                    (int16_t)(rect_y1 - rect_y0)
                };
                open = false;
            }
        }
    }

    if (open && count < max_rects) {
        out_rects[count++] = {
            (int16_t)(rect_bx0 * 4),
            (int16_t) rect_y0,
            (int16_t)((rect_bx1 - rect_bx0) * 4),
            (int16_t)(rect_y1 - rect_y0)
        };
    }

    return count;
}

// =============================================================================
// clearDirtyAreas()
// =============================================================================
void EPD_Painter::clearDirtyAreas(uint8_t* framebuffer, int tolerance, ClearMode mode) {
    // Compact 8bpp framebuffer into packed_paintbuffer (same as paint()).
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    if (_config.rotation == Rotation::ROTATION_CW)
        compact_pixels_rotated_cw(framebuffer, packed_paintbuffer, _config.height, _config.width);
    else if (_config.rotation == Rotation::ROTATION_180)
        compact_pixels_180(framebuffer, packed_paintbuffer, _config.width * _config.height);
    else
        epd_painter_compact_pixels(framebuffer, packed_paintbuffer, _config.width * _config.height);
    xSemaphoreGive(_paint_buffer_sem);

    Rect rects[32];
    int n = computeDirtyRects(rects, 32, tolerance);
    if (n > 0)
        clear(rects, n, mode);
}

// =============================================================================
// clear()
// Optional rects array restricts clearing to those regions only.
// =============================================================================
void EPD_Painter::clear(const Rect* rects, int num_rects, ClearMode mode) {

  const bool partial = (rects != nullptr && num_rects > 0);

  // NOTE (Tony, ghost hunt): clear() no longer auto-releases an active
  // template — a template now SURVIVES a full clear (the overlay simply
  // re-drives it on the next paint). Apps release explicitly.

  PanelPowerGuard guard(*this);

  const int prb = _config.width / 4;  // packed row bytes

  // Paint white into the affected area of the paint buffer. In 16-grey
  // mode the paint task reads the 4bpp paintbuffer, so whiten that too.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  if (!partial) {
    memset(packed_paintbuffer, 0x00, prb * _config.height);
    if (_grey16) memset(packed4_paintbuffer, 0x00, (size_t)prb * 2 * _config.height);
  } else {
    for (int r = 0; r < num_rects; r++) {
      int bx0 = rects[r].x / 4;
      int bx1 = (rects[r].x + rects[r].w + 3) / 4;
      if (bx1 > prb) bx1 = prb;
      int y0  = rects[r].y;
      int y1  = rects[r].y + rects[r].h;
      if (y1 > _config.height) y1 = _config.height;
      for (int row = y0; row < y1; row++) {
        memset(packed_paintbuffer + row * prb + bx0, 0x00, bx1 - bx0);
        if (_grey16)
          memset(packed4_paintbuffer + (size_t)row * prb * 2 + bx0 * 2, 0x00, (bx1 - bx0) * 2);
      }
    }
  }
  paintStage = 1;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 1) {
    vTaskDelay(1);
  }

  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  dma_buffer = dma_buffer1;

  // Hardware clear phases.
  int num_phases;
  int totpass[4];
  if (mode == ClearMode::SOFT) {
    num_phases = 2;
    totpass[0] = 1; totpass[1] = 1;
  } else {
    num_phases = 4;
    totpass[0] = 6; totpass[1] = 2; totpass[2] = 4; totpass[3] = 8;
  }
  for (int phase = 0; phase < num_phases; phase++) {

    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;

    for (int passes = 0; passes < totpass[phase]; passes++) {
      for (int row = 0; row < _config.height; ++row) {
        if (!partial) {
          memset(dma_buffer, pattern, prb);
        } else {
          memset(dma_buffer, 0x00, prb);
          for (int r = 0; r < num_rects; r++) {
            if (row >= rects[r].y && row < rects[r].y + rects[r].h) {
              int bx0 = rects[r].x / 4;
              int bx1 = (rects[r].x + rects[r].w + 3) / 4;
              if (bx1 > prb) bx1 = prb;
              memset(dma_buffer + bx0, pattern, bx1 - bx0);
            }
          }
          // The hardware phases must not scrub protected template pixels
          // (the delta unpaint above them is already overlay-protected).
          if (_has_template) {
            if (!_tpl_grey16) {
              const uint8_t *m = tpl_mask + (size_t)row * prb;
              for (int bx = 0; bx < prb; bx++)
                dma_buffer[bx] &= (uint8_t)~m[bx];
            } else {
              const uint8_t *m = tpl_mask + (size_t)row * prb * 2;
              for (int bx = 0; bx < prb; bx++) {
                const uint8_t a = m[bx * 2], b = m[bx * 2 + 1];
                uint8_t dm = 0;
                if (a & 0xF0) dm |= 0xC0;
                if (a & 0x0F) dm |= 0x30;
                if (b & 0xF0) dm |= 0x0C;
                if (b & 0x0F) dm |= 0x03;
                dma_buffer[bx] &= (uint8_t)~dm;
              }
            }
          }
        }
        sendRow(row == 0);
      }
      EPD_DELAY_MS(5);
    }
  }

  // Send neutral to close out all rows.
  memset(dma_buffer1, 0x00, prb);
  memset(dma_buffer2, 0x00, prb);
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

// =============================================================================
// packBuffer()
// =============================================================================
uint8_t* EPD_Painter::packBuffer(const uint8_t* fb) const {
    const size_t packed_size = (size_t)_config.width * _config.height / 4;
    uint8_t* buf = static_cast<uint8_t*>(
        heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
    if (!buf) return nullptr;
    if (_config.rotation == Rotation::ROTATION_CW)
        compact_pixels_rotated_cw(fb, buf, _config.height, _config.width);
    else if (_config.rotation == Rotation::ROTATION_180)
        compact_pixels_180(fb, buf, (uint32_t)_config.width * _config.height);
    else
        epd_painter_compact_pixels(fb, buf, (uint32_t)_config.width * _config.height);
    return buf;
}

// =============================================================================
// autoDetectBoard()
//
// AUTO preset fallback used by begin().
//
// If the user did not select a board explicitly, EPD_PAINTER_PRESET is built
// with placeholder pins and this function probes the known board I2C buses to
// find a matching preset at runtime.
//
// Returns true when a supported board is detected and _config has been replaced
// with the matching preset. Returns false when no known board responds, so
// begin() can fail cleanly instead of continuing with an invalid configuration.
// =============================================================================
bool EPD_Painter::autoDetectBoard() {
  #ifdef ARDUINO
    if (_config.data_pins[0] >= 0) return true;

    Rotation _rotation = _config.rotation;
    int i = 1;
    for (const auto& probe : Probe) {
      printf("[EPD] Probing board %d on SDA=%d, SCL=%d, addr=0x%x\n", i, probe.i2c_sda, probe.i2c_scl, probe.i2c_addr);
      TwoWire _w(1);   // use bus 1 just for the probe; will be deleted
      _w.begin(probe.i2c_sda, probe.i2c_scl, 100000);
      // Suppress ESP-IDF i2c_master NACK error logs — a NACK simply means
      // this board isn't here, which is expected for every non-matching probe.
      esp_log_level_set("i2c.master", ESP_LOG_NONE);
      _w.beginTransmission(probe.i2c_addr);
      bool found = (_w.endTransmission() == 0);
      esp_log_level_set("i2c.master", ESP_LOG_WARN);
      _w.end();
      // Reset Pins to output
      EPD_PIN_OUTPUT(probe.i2c_sda);
      EPD_PIN_OUTPUT(probe.i2c_scl);
      if(found) {
        printf("[EPD] Board %d found\n", i);
        _config = *probe.preset;
        _preset = probe.preset;
        _config.rotation = _rotation;  // preserve user-specified rotation across auto-detect
        printf("[EPD] Detected panel details: \n - I2C SDA: %d\n - I2C SCL: %d\n", _config.i2c.sda, _config.i2c.scl);
        return true;
      }
      ++i;
    }

    printf("[EPD] No known board found; begin() will fail\n");
    return false;
  
  #else
    return false;
  #endif
}
