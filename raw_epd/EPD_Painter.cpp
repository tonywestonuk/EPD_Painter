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

// Assembly routines — see EPD_Painter.S for full documentation

// Packs the GFX canvas (1 byte/pixel) down to 2 bits/pixel
extern "C" void epd_painter_compact_pixels(
  const uint8_t *input,
  uint8_t *output,
  uint32_t size);

// Converts a packed 2bpp row into the EPD ink drive format for one waveform pass
extern "C" uint32_t epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb,
  uint8_t *output,
  uint32_t length,
  const uint32_t *waveform);

// Copies pixels from the new framebuffer to the output, only where the screen
// buffer is currently white (00). Updates the screen buffer to reflect the change.
extern "C" uint32_t epd_painter_ink_on(
  const uint8_t *packed_src_fb,
  const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb,
  int16_t width,
  int16_t height,
  bool interlace_period);

// Copies pixels from the screen buffer to the output where they differ from the
// new framebuffer, clearing them in the screen buffer (returning them toward white).
extern "C" uint32_t epd_painter_ink_off(
  const uint8_t *packed_src_fb,
  const uint8_t *packed_cmp_fb,
  uint8_t *packed_out_fb,
  int16_t width,
  int16_t height,
  bool interlace_period);

// Copies every other row from source to destination (interleaved rows only)
extern "C" void epd_painter_interleaved_copy(
  const uint8_t *input,
  uint8_t *output,
  int16_t width,
  int16_t height,
  bool interlace_period);

// Number of waveform passes per frame. Each pass drives the panel with a
// different voltage pattern — the cumulative effect produces the target shade.
#define PASS_COUNT 6

// ---- LCD_CAM / DMA pin mapping ----
// Maps the 8 parallel data bits to their GPIO pins and LCD_CAM signal indices.
// The ordering here determines the bit-to-pin assignment on the data bus.
static const struct {
  int8_t pin;
  uint8_t signal;
} kDataPins[] = {
  { 6,  LCD_DATA_OUT0_IDX },
  { 14, LCD_DATA_OUT1_IDX },
  { 7,  LCD_DATA_OUT2_IDX },
  { 12, LCD_DATA_OUT3_IDX },
  { 9,  LCD_DATA_OUT4_IDX },
  { 11, LCD_DATA_OUT5_IDX },
  { 8,  LCD_DATA_OUT6_IDX },
  { 10, LCD_DATA_OUT7_IDX },
};

// =============================================================================
// Constructor
// =============================================================================
EPD_Painter::EPD_Painter()
  : GFXcanvas8(960, 540, false) {

  // Allocate the raw GFX canvas pixel buffer in PSRAM (1 byte per pixel).
  // This is the buffer that Adafruit_GFX drawing calls write into.
  uint32_t bytes = (uint32_t)width() * (uint32_t)height();
  buffer = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  if (buffer) memset(buffer, 0, bytes);
}

// =============================================================================
// sendRow() — DMA-transfer one packed row to the EPD via LCD_CAM
//
// The EPD panel uses a shift-register-based row interface:
//   SPV (frame start) — pulses low/high to reset the row pointer to line 0
//   CKV (row clock)   — falling edge advances to the next row
//   LE  (latch enable) — rising edge latches the current shift register into the row
//
// The DMA descriptors are set up as a circular pair (dma_desc1 ↔ dma_desc2),
// each pointing to one of two DMA buffers. We alternate between them each call
// so that the DMA engine is always streaming from one buffer while we prepare
// the next row in the other. This avoids stalling the DMA between rows.
//
// Sequence per row:
//   1. Swap which DMA buffer is "current" (the one just filled by the caller)
//   2. Wait for any previous DMA transfer to complete
//   3. Emit the row timing signals (SPV reset on first row, LE/CKV pulse otherwise)
//   4. Kick off the DMA transfer for this row
//   5. On the last row, wait for DMA to finish and latch the final row
// =============================================================================
void EPD_Painter::sendRow(bool firstLine, bool lastLine) {

  // Alternate between dma_buffer1 and dma_buffer2. The caller fills
  // whichever buffer dma_buffer points to; we swap here so the LCD_CAM
  // DMA descriptor chain reads from the buffer that was just filled.
  dma_buffer = dma_buffer == dma_buffer1 ? dma_buffer2 : dma_buffer1;

  // Wait for the previous row's DMA transfer to complete before touching
  // the timing signals (yield() lets other RTOS tasks run while we wait)
  while (LCD_CAM.lcd_user.lcd_start) {
    yield();
  }

  if (firstLine) {
    // Reset the panel's internal row pointer to line 0.
    // SPV low + CKV falling edge positions the shift register at the start.
    // SPV then goes high to allow normal row clocking to proceed.
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_SPV) | (1 << PIN_CKV));  // SPV=0, CKV=0
    delayMicroseconds(1);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));   // CKV rising edge
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_SPV));   // SPV=1 (normal operation)
  } else {
    // Latch the previous row's data into the panel, then advance to the next row.
    // LE rising edge latches the shift register contents into the display row.
    // CKV falling then rising edge clocks the row pointer forward by one.
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_LE));    // LE=1 (latch previous row)
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_CKV));   // CKV=0 (begin row advance)
    delayMicroseconds(5);                            // hold time required by panel
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));   // CKV=1 (row pointer advanced)
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_LE));    // LE=0 (latch complete)
  }

  // Start the DMA transfer — the LCD_CAM peripheral will clock out
  // packed_row_bytes bytes from the current DMA buffer onto the data bus.
  LCD_CAM.lcd_user.lcd_start = 1;

  if (lastLine) {
    // For the final row, we must wait for its DMA transfer to complete
    // before issuing the latch pulse — otherwise the last row's data
    // would not yet be in the shift register when we try to latch it.
    while (LCD_CAM.lcd_user.lcd_start) {
      yield();
    }

    // Latch and advance row clock one final time to commit the last row
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_LE));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_CKV));
    delayMicroseconds(5);
    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_LE));
  }
}

// =============================================================================
// begin() — Initialise hardware and allocate all buffers
// =============================================================================
bool EPD_Painter::begin() {

  // ---- Configure EPD control pins as outputs ----
  pinMode(PIN_PWR, OUTPUT);  // Panel power supply enable
  pinMode(PIN_SPV, OUTPUT);  // Frame start (vertical sync)
  pinMode(PIN_CKV, OUTPUT);  // Row clock (vertical clock)
  pinMode(PIN_SPH, OUTPUT);  // Line start (horizontal sync)
  pinMode(PIN_OE,  OUTPUT);  // Output enable
  pinMode(PIN_LE,  OUTPUT);  // Latch enable (commits row data)
  pinMode(PIN_CL,  OUTPUT);  // Pixel clock (driven by LCD_CAM PCLK)

  // packed_row_bytes: number of bytes in one packed (2bpp) row.
  // At 2 bits per pixel, a row of N pixels takes N/4 bytes.
  packed_row_bytes = width() / 4;

  // ---- Enable and reset the LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(100);

  // ---- Configure the LCD_CAM pixel clock ----
  // Source: PLL_F160M (160 MHz)
  // Dividers: lcd_clkm_div_num=1 → ÷2 → 80 MHz, lcd_clkcnt_n=1 → ÷2 → 40 MHz
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;          // Clock source: PLL_F160M_CLK
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;      // PCLK idles low
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;     // PCLK idles low between transfers
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;  // Use divider (not pass-through)
  LCD_CAM.lcd_clock.lcd_clkm_div_num = 1;    // ÷2 first stage
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;      // No fractional division
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;        // ÷2 second stage

  // ---- Configure LCD_CAM for i8080 8-bit parallel mode ----
  // Disable RGB mode (we're driving i8080, not an RGB panel)
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
  // Bypass YUV colour conversion — we send raw pixel bytes
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;
  // Don't auto-restart frame — we trigger each row manually
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  // No output data bit reordering or delays
  LCD_CAM.lcd_data_dout_mode.val = 0;
  LCD_CAM.lcd_user.lcd_always_out_en = 0;
  LCD_CAM.lcd_user.lcd_8bits_order = 0;      // Standard byte order
  LCD_CAM.lcd_user.lcd_bit_order = 0;        // MSB first
  LCD_CAM.lcd_user.lcd_2byte_en = 0;         // 8-bit mode (not 16-bit)
  LCD_CAM.lcd_user.lcd_dummy = 0;            // No dummy cycles
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
  LCD_CAM.lcd_user.lcd_cmd = 0;              // No command phase

  // Set the number of data bytes per transfer (one packed row), 0-indexed
  LCD_CAM.lcd_user.lcd_dout_cyclelen = packed_row_bytes - 1;
  LCD_CAM.lcd_user.lcd_dout = 1;   // Enable data output
  LCD_CAM.lcd_user.lcd_update = 1; // Apply configuration

  // ---- Route the 8-bit data bus to GPIO pins via the LCD_CAM signal matrix ----
  for (int i = 0; i < 8; i++) {
    esp_rom_gpio_connect_out_signal(kDataPins[i].pin, kDataPins[i].signal, false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[kDataPins[i].pin], PIN_FUNC_GPIO);
    gpio_set_drive_capability((gpio_num_t)kDataPins[i].pin, (gpio_drive_cap_t)3); // max drive
  }

  // ---- Route the pixel clock (PCLK) to the CL pin ----
  esp_rom_gpio_connect_out_signal(PIN_CL, LCD_PCLK_IDX, false, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_CL], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)PIN_CL, (gpio_drive_cap_t)3);

  // ---- Allocate and configure a GDMA channel to feed the LCD_CAM peripheral ----
  gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,   // Memory → peripheral
    .flags = { .reserve_sibling = 0 },
  };
  gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  // Disable owner-check and auto descriptor update — we manage descriptors manually
  gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false,
  };
  gdma_apply_strategy(dma_chan, &strategy_config);

  // ---- Allocate two DMA row buffers in internal DMA-capable RAM ----
  // We double-buffer so the CPU can fill one buffer while DMA streams the other.
  dma_buffer1 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));

  dma_buffer = dma_buffer1;  // start with buffer 1 as the "current" write target

  // ---- Set up circular DMA descriptor chain: desc1 → desc2 → desc1 → ... ----
  // Each descriptor points to one row buffer. The LCD_CAM peripheral automatically
  // moves to the next descriptor when one transfer completes. By making them
  // circular and alternating which buffer we fill, we achieve continuous streaming.
  dma_desc2.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc2.dw0.suc_eof = 0;                    // Not end-of-frame
  dma_desc2.dw0.size = packed_row_bytes;
  dma_desc2.dw0.length = packed_row_bytes;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2);

  dma_desc1.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
  dma_desc1.dw0.suc_eof = 0;
  dma_desc1.dw0.size = packed_row_bytes;
  dma_desc1.dw0.length = packed_row_bytes;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1);

  dma_desc1.next = &dma_desc2;   // desc1 → desc2
  dma_desc2.next = &dma_desc1;   // desc2 → desc1 (circular)

  // Reset the LCD FIFO and start the DMA engine pointing at descriptor 1
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  gdma_start(dma_chan, (intptr_t)&dma_desc1);

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = (width() * height()) / 4;  // 2 bpp → 4 pixels per byte

  // packed_fastbuffer: working buffer in internal RAM for fast per-row access
  // during the waveform drive loop (avoids slow PSRAM reads in the hot path)
  packed_fastbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));

  // packed_screenbuffer: persistent record of what is currently displayed on
  // the physical panel. Stored in PSRAM as it's only accessed once per frame.
  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  // Both start as all-white (0x00 = all pixels white in 2bpp format)
  memset(packed_screenbuffer, 0x00, packed_size);
  memset(getBuffer(), 0x00, width() * height());

  return (dma_buffer && packed_fastbuffer && packed_screenbuffer);
}

// =============================================================================
// end() — Release hardware resources
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
  // Hold SPH and SPV low before enabling power
  digitalWrite(PIN_SPV, LOW);
  digitalWrite(PIN_SPH, LOW);
  digitalWrite(PIN_OE,  HIGH);   // Enable output drivers
  delayMicroseconds(100);
  digitalWrite(PIN_PWR, HIGH);   // Enable panel power supply
  delayMicroseconds(100);

  // Issue the initial frame-start pulse to position the row pointer at line 0.
  // SPV low + CKV low → CKV rising edge → SPV high arms normal row clocking.
  REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_SPV) | (1 << PIN_CKV));
  delayMicroseconds(1);
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_CKV));
  REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_SPV));
}

void EPD_Painter::powerOff() {
  digitalWrite(PIN_PWR, LOW);    // Cut panel power supply
  delayMicroseconds(100);
  digitalWrite(PIN_OE,  LOW);    // Disable output drivers
}

// =============================================================================
// Waveform tables
//
// E-paper displays achieve greyscale by driving each pixel with a sequence
// of positive, negative, or zero voltage pulses across multiple passes.
// Each entry [row][pass] is a 2-bit drive code per pixel-value group:
//   0 = no drive (float)
//   1 = drive positive (whiten)
//   2 = drive negative (darken)
//   3 = drive both simultaneously (used for DC-balancing)
//
// lighter_waveform: applied to rows that should move toward white/lighter shades
// darker_waveform:  applied to rows that should move toward black/darker shades
//
// The interlaced drive scheme applies lighter to even rows and darker to odd
// rows (or vice versa, alternating each frame) so that all pixels eventually
// settle at the correct shade through the accumulated drive history.
//
// Row index:  0 = pixels currently black/darkest
//             1 = pixels currently dark grey
//             2 = pixels currently light grey
// =============================================================================
static const uint8_t lighter_waveform[][6] = {
  { 1, 2, 2, 2, 3, 0 },   // for 11 pixels (black) → lighten over 6 passes
  { 1, 2, 2, 2, 3, 2 },   // for 10 pixels (dark grey)
  { 2, 2, 2, 2, 2, 2 }    // for 01 pixels (light grey)
};

static const uint8_t darker_waveform[][6] = {
  { 1, 1, 0, 0, 0, 0 },   // for 11 pixels (black)
  { 1, 1, 1, 0, 0, 0 },   // for 10 pixels (dark grey)
  { 1, 1, 1, 1, 1, 1 }    // for 01 pixels (light grey) → darken over 6 passes
};

// =============================================================================
// paint() — Push the current GFX canvas to the physical display
//
// Overview:
//   1. Compact the GFX canvas (8 bits/pizel → 2bits per packed) into packed_fastbuffer
//   2. ink_on:  identify pixels that need to turn on (were white, now grey/black)
//               → copy their new values into packed_fastbuffer for driving,
//               → mark them in screen buffer (they are now "inked")
//   3. ink_off: identify pixels that need to change shade (differ from screen)
//               → copy their current screen values into packed_fastbuffer for driving,
//               → clear them in screen buffer (they must return to white first)
//   4. Drive 6 waveform passes:
//               Each pass converts packed_fastbuffer to ink drive format and
//               streams it row by row via DMA. Alternate rows get the "darker"
//               waveform (interlaced rows), the rest get "lighter".
//   5. Neutralise: send a full frame of zero drive (0x00) to stop driving all pixels.
//   6. Toggle interlace_period for the next frame.
// =============================================================================
void EPD_Painter::paint() {
  PanelPowerGuard guard(*this);   // Powers on panel; powers off automatically on scope exit
  const int packed_row_bytes = width() / 4;

  // Step 1: Pack the 8bpp GFX canvas down to 2bpp
  epd_painter_compact_pixels(buffer, packed_fastbuffer, width() * height());

  // Step 2: Mark pixels turning on (white → grey/black).
  // ink_on writes the new pixel values to packed_fastbuffer only where the
  // screen buffer shows white (00). It operates on even rows (interlace_period).
  epd_painter_ink_on(
    packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
    packed_row_bytes, height(), interlace_period);

  // Step 3: Mark pixels turning off (grey/black → different shade or white).
  // ink_off writes the current screen values to packed_fastbuffer where the
  // desired value differs from what's on screen, then clears those pixels
  // in the screen buffer. Operates on odd rows (!interlace_period).
  epd_painter_ink_off(
    packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
    packed_row_bytes, height(), !interlace_period);

  // Step 4: Drive 6 waveform passes
  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {

    // Build the drive words for this pass by combining the 3 waveform bytes
    // (one per pixel value group: 11, 10, 01) into a single uint32_t.
    // Multiplied by 0x55 to broadcast the 2-bit pattern to all byte lanes.
    const uint32_t lighter_fmt =
      ((lighter_waveform[0][pass] << 16) +
       (lighter_waveform[1][pass] << 8)  +
        lighter_waveform[2][pass]) * 0x55;

    const uint32_t darker_fmt =
      ((darker_waveform[0][pass] << 16) +
       (darker_waveform[1][pass] << 8)  +
        darker_waveform[2][pass]) * 0x55;

    for (int row = 0; row < height(); row++) {
      // Alternate between darker and lighter waveforms on odd/even rows
      // to implement interlaced driving. Which parity gets "darker" flips
      // each frame via interlace_period.
      uint32_t waveform = (row % 2 == interlace_period) ? darker_fmt : lighter_fmt;

      // Convert this row's packed 2bpp data into the ink drive format
      // and place it directly into the DMA buffer for this row
      epd_painter_convert_packed_fb_to_ink(
        packed_fastbuffer + row * packed_row_bytes,
        dma_buffer,
        packed_row_bytes,
        &waveform);

      sendRow(row == 0);   // Transmit row; first row triggers SPV frame-start pulse
    }
  }

  // Step 5: Neutralise — send a full frame of zero drive to stop all pixel movement.
  // Both DMA buffers are zeroed so no voltage is applied during this sweep.
  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < height(); ++row) {
    sendRow(row == 0, row == height() - 1);
  }

  // Step 6: Flip interlace polarity for the next frame
  interlace_period = !interlace_period;
}

// =============================================================================
// clear() — Erase the display to all white
//
// Overview:
//   1. Set packed_fastbuffer to all-white (0x00)
//   2. Call ink_off twice (once per interlace parity) to discharge any pixels
//      currently showing grey/black in the screen buffer
//   3. Drive 6 lighter waveform passes to push all pixels toward white
//   4. Drive 4 passes of alternating hard white (01010101) and hard black
//      (10101010) to fully reset the electrophoretic particles
//   5. Neutralise with a zero-drive sweep
// =============================================================================
void EPD_Painter::clear() {
  PanelPowerGuard guard(*this);
  const int packed_row_bytes = width() / 4;

  // Target state is all-white (0x00 in 2bpp = all pixels are 00)
  memset(packed_fastbuffer, 0x00, height() * packed_row_bytes);

  // Use ink_off for both interlace parities to mark all currently-inked pixels
  // as needing to be discharged (they differ from the all-white target)
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
                      packed_row_bytes, height(), true);
  epd_painter_ink_off(packed_fastbuffer, packed_screenbuffer, packed_fastbuffer,
                      packed_row_bytes, height(), false);

  // Drive 6 lighter waveform passes to discharge, and move all grey/black pixels toward white
  for (uint8_t pass = 0; pass < PASS_COUNT; pass++) {
    const uint32_t lighter_fmt =
      ((lighter_waveform[0][pass] << 16) +
       (lighter_waveform[1][pass] << 8)  +
        lighter_waveform[2][pass]) * 0x55;

    for (int row = 0; row < height(); row++) {
      epd_painter_convert_packed_fb_to_ink(
        packed_fastbuffer + row * packed_row_bytes,
        dma_buffer,
        packed_row_bytes,
        &lighter_fmt);
      sendRow(row == 0);
    }
  }

  // Hard erase: alternate between full-white drive (01010101) and full-black
  // drive (10101010) for 4 passes each. This aggressively resets any stubborn
  // particles that haven't responded to the waveform passes above.
  for (int phase = 0; phase < 2; phase++) {
    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;
    memset(dma_buffer1, pattern, packed_row_bytes);
    memset(dma_buffer2, pattern, packed_row_bytes);

    for (int passes = 0; passes < 4; passes++) {
      for (int row = 0; row < height(); ++row) {
        sendRow(row == 0);
      }
    }
  }

  // Neutralise: zero-drive sweep to stop all pixel movement
  memset(dma_buffer1, 0x00, packed_row_bytes);
  memset(dma_buffer2, 0x00, packed_row_bytes);
  for (int row = 0; row < height(); ++row) {
    sendRow(row == 0, row == height() - 1);
  }
}