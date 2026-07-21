#ifndef EPD_Painter_H
#define EPD_Painter_H

#include <stddef.h>
#include <stdint.h>

// Pin encoding: set bit 8 to indicate the pin lives on the 74HCT4094D shift
// register.  Bits 0–7 hold the output index (QP0..QP7).
// Example: EPD_SR_PIN(4) → shift-register output QP4 (EP_STV/SPV on H752).
// A value of -1 means the pin is absent / managed elsewhere.
#define EPD_SR_PIN(n)  (int16_t(0x100 | (n)))

static inline bool    epd_pin_is_sr(int16_t p)  { return (p & 0x100) != 0; }
static inline uint8_t epd_pin_sr_bit(int16_t p) { return uint8_t(p & 0xFF); }

#if !defined(EPD_PAINTER_PRESET_M5PAPER_S3)       && \
    !defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) && \
    !defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752) && \
    !defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716)
  
  #ifndef EPD_PAINTER_PRESET_AUTO
    #define EPD_PAINTER_PRESET_AUTO
  #endif
#endif

// Forward declarations — avoid circular includes
class EPD_PainterShutdown;
class EPD_PinDriver;
class EPD_PowerDriver;
class EPD_ISRController;

// FreeRTOS headers — available in both Arduino-ESP32 and pure ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <atomic>


#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <esp_intr_alloc.h>

// I2C — TwoWire for Arduino only; ESP-IDF builds don't use I2C
#ifdef ARDUINO
  #include <Wire.h>
#endif


class EPD_Painter {

public:

struct I2CBusConfig {
#ifdef ARDUINO
    TwoWire* wire = nullptr;
#else
    void* i2c_bus = nullptr;  // unused in ESP-IDF builds
#endif
    int sda = -1;
    int scl = -1;
    uint32_t freq = 100000;
};
struct PowerCtlConfig {
    int pca_addr=-1;
    int tps_addr=-1;
};
  enum class Quality {
    QUALITY_HIGH,
    QUALITY_NORMAL,
    QUALITY_FAST
  };

  enum class Rotation {
    ROTATION_0,   // landscape — normal orientation
    ROTATION_CW,  // 90° clockwise — portrait drawing canvas (width↔height swapped)
    ROTATION_180  // landscape, flipped — same dims as ROTATION_0, image reversed at pack
  };

  struct Waveforms {
      uint8_t fast_lighter[3][7];
      uint8_t fast_darker[3][7];
      uint8_t normal_lighter[3][13];
      uint8_t normal_darker[3][13];
      uint8_t high_lighter[3][13];
      uint8_t high_darker[3][13];
  };

  // Optional temperature-compensated waveform set: used when the panel
  // temperature (from the power PMIC's sensor) is below `below_c`. Cold ink
  // is more viscous and needs stronger drive to reach the same grey levels.
  struct WaveformBand {
      int8_t below_c;
      Waveforms waveforms;
  };
  static constexpr int MAX_WAVEFORM_BANDS = 3;

  struct Shift {
    int8_t data    = -1;
    int8_t clk     = -1;
    int8_t strobe  = -1;
    int8_t le_time = 0;
    enum Driver { H752, H716 } driver = H752;
  };


  struct Config {
      uint16_t width;
      uint16_t height;
      int16_t pin_pwr;
      int16_t pin_syspwr = -1;
      int16_t pin_sph;
      int16_t pin_oe;
      int16_t pin_cl;
      int16_t pin_spv;
      int16_t pin_ckv;
      int16_t pin_le;
      Quality  quality;
      Rotation rotation = Rotation::ROTATION_0;

      // Dummy zero bytes clocked out after each row's pixel data (4 bytes =
      // 16 pixel clocks). The panel's source-driver shift chain is slightly
      // longer than the visible width; without this flush the last ~16
      // columns latch data belonging to pixels further left, repeating the
      // image at the right-hand edge. See How_It_Works.md §7.
      uint8_t row_pad_bytes = 4;
      int8_t data_pins[8];
      I2CBusConfig i2c{};
      PowerCtlConfig power{};
      Waveforms waveforms;

      // Cold-temperature waveform bands, sorted ascending by below_c. At each
      // panel power-on the panel temperature picks the first band whose
      // below_c exceeds it; if none match (or no temp sensor), the default
      // `waveforms` table is used.
      WaveformBand waveform_bands[MAX_WAVEFORM_BANDS];
      uint8_t num_waveform_bands = 0;

      Shift shift;

      // 16-grey constant pass period per quality (microseconds). The pass
      // loop pads every pass to this so retention dose depends only on the
      // trains, and the trains are calibrated AT this period — so it is
      // per-board: it must exceed the board's worst-case row loop plus the
      // quality's settle floor. Boards whose control lines ride a shift
      // register (H716) have much slower rows and need a longer period.
      int g16_pass_us_normal = 15000;
      int g16_pass_us_high   = 19000;

      // Returns a copy of this config with rotation set — lets you write:
      //   EPD_PainterAdafruit epd(EPD_PAINTER_PRESET.withRotation(EPD_Painter::Rotation::ROTATION_CW));
      Config withRotation(Rotation r) const { Config c = *this; c.rotation = r; return c; }
  };

  struct ProbeSettings {
    Config *preset;
    int i2c_sda;
    int i2c_scl;
    int i2c_addr;
    bool found = false;
  };

  Config _config;
  const Config* _preset = nullptr;

  EPD_Painter(const Config &config, bool portrait = false);
  bool begin();
  bool end();

  struct Rect {
    int16_t x, y, w, h;
  };

  enum class ClearMode {
    HARD,  // 4 phases {6,2,4,8} passes — full ghosting removal
    SOFT   // 2 phases {2,2} passes — faster, light refresh
  };

  void clear(const Rect* rects = nullptr, int num_rects = 0, ClearMode mode = ClearMode::HARD);

  // Compare screenbuffer (current display state) against paintbuffer (desired state)
  // and return dirty rectangles. tolerance = max clean pixels a rect may absorb
  // when bridging gaps between dirty rows (0 = merge only adjacent dirty rows,
  // larger values merge more, screen_width * screen_height = single rect).
  int computeDirtyRects(Rect* out_rects, int max_rects, int tolerance = 0) const;

  // Convenience: compact framebuffer → paintbuffer, compute dirty rects, then
  // clear only those areas. tolerance is passed to computeDirtyRects.
  void clearDirtyAreas(uint8_t* framebuffer, int tolerance = 0, ClearMode mode = ClearMode::SOFT);

  // TEMPORARY hardware probe: manually drives a three-band stripe pattern
  // with raw drive codes to establish whether one gate row can be latched
  // and driven twice (LE without CKV). See implementation for band layout.
  void debugRowTest();

  void fxClear();
  void clearBuffers();  // zero all packed buffers (call before power-off to reset DC-balance baseline)
  void paint(uint8_t* framebuffer);

  // Paint from a pre-packed 2bpp buffer (4 pixels/byte, first pixel in the
  // MSBs). line_repeat > 1 treats `packed` as a reduced-height buffer of
  // height/line_repeat rows and drives each source row line_repeat times —
  // e.g. line_repeat=2 paints a 960×270 buffer as 960×540, halving the
  // bandwidth needed when streaming frames from storage. height must be an
  // exact multiple of line_repeat.
  void paintPacked(const uint8_t* packed, int line_repeat = 1);
  void unpaintPacked(const uint8_t* packed);
  void paintLater(uint8_t* framebuffer);

  // Completed drive cycles since begin(). paintLater() drops frames when
  // submissions outpace the panel (only the latest render is painted), so
  // effective paint rate = delta of this counter over wall time, never the
  // submit-loop rate. No-op cycles (nothing changed) are not counted.
  uint32_t paintsCompleted() const { return _paints_done.load(); }
  bool paintIdle() const { return paintStage.load() == 0; }
  // No-op, kept for API compatibility. Interlace mode existed to spatially
  // dither the old either-direction-per-chunk deferral artifact; the
  // dual-plane ink engine drives both directions per pixel in one cycle,
  // so there is nothing left to interlace.
  void setInterlaceMode(bool) {}

  void setQuality(Quality quality);

  // Select the paint engine: false (default) = SIMD dual-plane assembly,
  // true = the decision engine's C discovery path (see DECISION_ENGINE.md).
  // Phase B: both produce identical output on 4-level content; the C path
  // exists to be verified against the assembly before it grows 16 levels.
  void setDecisionEngine(bool on) { _decision_engine = on; }

  // 16-grey mode (decision engine phase C — see DECISION_ENGINE.md).
  // levels = 4 (default) or 16. While 16-grey is on, paint() reads the 8bpp
  // canvas as level codes 0..15 (0 = white … 15 = black; use dither16() for
  // greyscale sources) and paintPacked()/unpaintPacked() are refused — the
  // packed 2bpp path stays a 4-level feature. NORMAL and HIGH only: FAST's
  // 7 undelayed passes lack the dose resolution, so the call fails while
  // quality is FAST. First enable allocates the 4bpp state and spill slot
  // planes (~1.6 MB PSRAM). The physical screen state is carried across the
  // switch in both directions, so ghost tracking survives without a clear.
  // Call after begin(). Returns false on refusal or allocation failure.
  bool setGreyLevels(int levels);

  // Override one 16-grey decision train (id = (level << 1) | dir, dir 0 =
  // apply / 1 = remove; train = DEC_WF_LEN16 drive codes). The calibration
  // rig's hook: probe sketches load candidate trains and the scanner
  // measures what the glass actually does with them. Only meaningful after
  // setGreyLevels(16) has built the default library.
  void setDecisionTrain(uint8_t id, const uint8_t *train) {
    if (id >= 2 && id < DEC_IDS && train)
      for (int p = 0; p < DEC_WF_LEN16; p++) dec_trains16[id][p] = train[p];
  }

  // Direct grey-to-grey transitions (4-level mode — see DECISION_ENGINE.md
  // "direct grey-to-grey transitions"). When on, a changed occupied pixel
  // whose (from, to) pair has a loaded train is driven straight to its new
  // level in ONE paint — no erase-to-white two-step. Pairs without a train
  // keep the legacy two-step, so the engine comes up one tuned pair at a
  // time. First enable allocates 2 spill slot planes (~260 KB PSRAM).
  // Call after begin(). Returns false on allocation failure.
  bool setDirectTransitions(bool on);

  // Decision id of a direct transition: from/to = 2bpp levels 1..3.
  static constexpr uint8_t directId(uint8_t from, uint8_t to) {
    return (uint8_t)(16 | (from << 2) | to);
  }

  // Load one direct train (len drive codes, up to DEC_WF_LEN_DIR = 26 —
  // a direct train may be LONGER than the quality's pass count: deep
  // lightening transitions need erase + re-darken room, and the frame's
  // pass count extends to the longest direct train in play, so content
  // without deep transitions pays nothing). nullptr unloads the pair back
  // to the two-step. The DC constraint is the tuner's job, not enforced
  // here: the train's net darkens must equal Q(to) - Q(from), where Q(g)
  // is the net darkens of level g's apply train — that makes the charge
  // ledger path-independent.
  void setDirectTrain(uint8_t from, uint8_t to, const uint8_t *train,
                      int len = DEC_WF_LEN16) {
    if (from < 1 || from > 3 || to < 1 || to > 3 || from == to) return;
    const int key = (from << 2) | to;
    if (!train) { _dir_loaded &= (uint16_t)~(1u << key); return; }
    if (len > DEC_WF_LEN_DIR) len = DEC_WF_LEN_DIR;
    for (int p = 0; p < DEC_WF_LEN_DIR; p++)
      dec_trains_dir[key][p] = (p < len) ? train[p] : 0;
    _dir_loaded |= (uint16_t)(1u << key);
  }

  // Static template layer (Tony's banded-quality pattern, July 2026).
  // setTemplate() paints fb (8bpp canvas, same format as paint()) at the
  // given quality — independent of the current quality — then PROTECTS
  // its non-white pixels: no subsequent paint can touch them, so
  // animation runs in the template's white areas at any speed while the
  // template keeps its high-quality rendering. (Unchanged pixels are
  // never re-driven anyway; protection turns that from app discipline
  // into a guarantee.) releaseTemplate() unpaints the template with the
  // SAME quality that painted it — a NORMAL pixel erased by a FAST
  // train would strand charge — and lifts the protection. clear()
  // auto-releases first; setGreyLevels() is refused while a template is
  // active. ~260 KB PSRAM (double in 16-grey). Returns false on
  // allocation failure or FAST+16-grey.
  bool setTemplate(const uint8_t *fb, Quality quality);
  void releaseTemplate();
  bool hasTemplate() const { return _has_template; }

  // Panel temperature in °C from the power PMIC's sensor (TPS65185 boards).
  // Returns EPD_PowerDriver::TEMP_UNAVAILABLE (-1000) if the board has no
  // sensor or begin() has not run yet.
  int readPanelTemperatureC();

  // Dither an 8bpp framebuffer (0=black … 255=white) in-place to the driver's
  // 4-level encoding: 0=white, 1=lt grey, 2=dk grey, 3=black.
  // Uses Floyd-Steinberg error diffusion. Allocates one row of int16 scratch
  // (~2 KB for a 960-wide panel) internally.
  static void dither(uint8_t* fb, uint16_t width, uint16_t height);

  // Pack an 8bpp framebuffer to the driver's 2bpp packed format, respecting
  // the current rotation setting. Returns a PSRAM-allocated buffer the caller
  // must free with heap_caps_free(). Returns nullptr on allocation failure.
  // Use this to obtain a packed buffer suitable for EPD_BootCtl::IImageProvider.
  uint8_t* packBuffer(const uint8_t* fb) const;

  const Config& getConfig(){
    return _config;
  }
  const Config* getPreset() const {
    return _preset;
  }

  void setAutoShutdown(bool v) { _autoShutdown = v; }
  EPD_PainterShutdown* shutdown() { return _shutdown; }


private:
  // ---- LCD_CAM / DMA ----
  gdma_channel_handle_t dma_chan = nullptr;
  int                   _dma_channel_id = 0;
  dma_descriptor_t      dma_desc1 = {};
  dma_descriptor_t      dma_desc2 = {};
  intr_handle_t         _lcd_intr_handle = nullptr;
  volatile TaskHandle_t _dma_notify_task = nullptr;
  bool                  _dma_pending = false;

  static void IRAM_ATTR _lcd_isr(void *arg);

  // ---- Buffers ----

  uint8_t* dma_buffer        = nullptr;  // Points at one of the buffers below
  uint8_t* dma_buffer1       = nullptr;  //  Row Double buffer A
  uint8_t* dma_buffer2       = nullptr;  //. Row Double buffer B

  uint8_t* packed_fastbuffer  = nullptr;  // dark-plane drive data (internal RAM)
  uint8_t* packed_lightbuffer = nullptr;  // light-plane drive data (PSRAM, sparse)
  uint8_t* packed_screenbuffer = nullptr; // 2bpp physical screen state (PSRAM)
  uint8_t* packed_paintbuffer = nullptr; // 2bpp desired frame (PSRAM)

  uint32_t* bitmask = nullptr;        // per-row dark-plane chunk mask
  uint32_t* bitmask_light = nullptr;  // per-row light-plane chunk mask

  // ---- Decision engine (see DECISION_ENGINE.md) ----
  // A decision is a (grey level, direction) pair, id = (level << 1) | dir
  // (dir 0 = apply, 1 = remove). Discovery fills per-line todo words and
  // sweep lists; the pass loop consumes the lists generically. Phase B:
  // 4-level compatibility — decisions 2..7, at most 2 sweeps per line.
  // The direct-transition engine adds ids 16..30 (16 | (from << 2) | to)
  // and can put 12 distinct decisions on a line (3 applies + 3 removes +
  // 6 directs) = 4 sweeps; compat still writes at most 2.
  static constexpr int DEC_MAX_SWEEPS   = 4;   // 4-level sweep lists
  static constexpr int DEC_MAX_SWEEPS16 = 10;  // 16-grey: ceil(30 decisions / 3)
  static constexpr int DEC_IDS          = 32;  // 16 levels x 2 directions
  static constexpr int DEC_WF_LEN16     = 13;  // train length (NORMAL/HIGH)
  static constexpr int DEC_WF_LEN_DIR   = 26;  // max direct-train length
  // 16-grey constant pass period (row loop + padding), per quality. A row
  // converts k sweeps, so the row loop's duration varies with content —
  // and pass duration IS the retention dose. Padding every pass to a
  // fixed period makes dose depend only on the trains (phase D; see
  // DECISION_ENGINE.md "dose is content-dependent").
  static constexpr int DEC_PASS_US16_NORMAL = 15000;
  static constexpr int DEC_PASS_US16_HIGH   = 19000;
  struct LineSweep {
    const uint8_t *plane_row;  // slot-encoded 2bpp row
    uint32_t       mask;       // chunk mask for this row
    uint8_t        dec[3];     // decision id per slot 1..3 (0 = unused)
  };
  uint32_t  *dec_todo    = nullptr;  // per-line pending-decision words
  LineSweep *dec_sweeps  = nullptr;  // [height * DEC_MAX_SWEEPS]
  uint8_t   *dec_nsweeps = nullptr;  // per-line sweep count
  const uint8_t *dec_train[DEC_IDS]; // decision id -> waveform train (wf_len codes)
  // C discovery: paintbuffer vs screenbuffer -> planes, masks, todo, sweep
  // lists; updates screenbuffer. Returns nonzero if any line has work.
  uint32_t _decision_discover();
  uint32_t _decision_discover_row(const uint8_t *pb_row, uint8_t *fbD_row,
                                  uint8_t *fbL_row, uint8_t *sb_row,
                                  uint32_t *maskL_out, uint32_t *todo_out);
  void _decision_batch_compat();
  bool _decision_engine = false;   // C discovery path off until phase C needs it

  // ---- direct grey-to-grey transitions (4-level; DECISION_ENGINE.md) ----
  // key = (from << 2) | to, decision id = 16 | key. _dir_loaded gates
  // per-pair engagement so unloaded pairs fall back to the two-step.
  uint8_t   dec_trains_dir[16][DEC_WF_LEN_DIR];
  uint16_t  _dir_loaded = 0;
  bool      _decision_direct = false;
  uint8_t  *dec_spill_dir = nullptr;   // slot planes for sweeps 2..3 (PSRAM)
  uint32_t _decision_discover_direct();
  void _decision_discover_direct_row(int row);
  uint8_t *_dec_plane_row_dir(int sweep, int row);

  // ---- 16-grey mode (phase C) ----
  // 4bpp desired/physical state, spill slot planes for sweeps 2..9 (sweep 0
  // = fastbuffer, sweep 1 = lightbuffer), wide sweep lists, and a formula
  // train library (placeholder — calibration is phase D). All PSRAM,
  // allocated lazily on the first setGreyLevels(16).
  uint8_t   *packed4_paintbuffer  = nullptr;  // w*h/2
  uint8_t   *packed4_screenbuffer = nullptr;  // w*h/2
  uint8_t   *dec_spill            = nullptr;  // (DEC_MAX_SWEEPS16-2) slot planes
  LineSweep *dec_sweeps16         = nullptr;  // [height * DEC_MAX_SWEEPS16]
  uint8_t    dec_trains16[DEC_IDS][DEC_WF_LEN16];
  // Temporal partition state: dec_gg_from[to] = bitmask of from-levels
  // that transitioned to `to` this frame (set by discovery). Each apply
  // id shifts by the longest remove among ITS OWN from-partners — not a
  // global R — into the per-frame dec_shifted scratch.
  uint16_t   dec_gg_from[16];
  uint8_t    dec_shifted[DEC_IDS][DEC_WF_LEN_DIR];
  bool       _grey16 = false;
  uint32_t _decision_discover16();
  void _decision_discover16_row(int row);
  uint8_t *_dec_plane_row(int sweep, int row);
  void _grey16_build_trains();

  // ---- template layer ----
  // Packed template + protection mask (11 / 1111 fields where a template
  // pixel is non-white), stored for the mode active at setTemplate().
  uint8_t *tpl_data = nullptr;
  uint8_t *tpl_mask = nullptr;
  bool     _has_template = false;
  bool     _tpl_grey16   = false;
  Quality  _tpl_quality  = Quality::QUALITY_NORMAL;

  int packed_row_bytes = 0;
  std::atomic<int> paintStage{0};
  std::atomic<uint32_t> _paints_done{0};
  bool shouldSkipRow = false;
  bool _autoShutdown = true;
  EPD_PainterShutdown* _shutdown = nullptr;


  // Waveform table in use — &_config.waveforms or a matched temperature
  // band's table; re-selected at every panel power-on.
  const Waveforms* _active_wf = nullptr;
  void selectWaveformsForTemperature();

  // ---- Hardware drivers (created in begin()) ----
  EPD_PowerDriver*   _powerDriver = nullptr;
  EPD_ISRController* _shiftReg    = nullptr;  // non-null only on SR boards
  EPD_PinDriver* _pin_spv = nullptr;
  EPD_PinDriver* _pin_ckv = nullptr;
  EPD_PinDriver* _pin_le  = nullptr;
  EPD_PinDriver* _pin_sph = nullptr;

  // ---- Internal helpers ----
  void powerOn();
  void powerOff();
  bool autoDetectBoard();
  // noAdvance: pulse LE to latch the previously transmitted data onto the
  // source outputs but do NOT clock CKV — the gate stays on the current row.
  // Used to drive one row with two data sets (dark plane, then light plane).
  void sendRow(bool firstLine, bool lastLine=false, bool noAdvance=false);

  // ---- Dual-core paint task ----
  SemaphoreHandle_t _paint_active_sem = nullptr;  // signals task to start
  SemaphoreHandle_t _paint_buffer_sem  = nullptr;  // signals task has finished
  TaskHandle_t      _paint_task_h    = nullptr;

  static void _paint_task_entry(void *arg);
  void _paint_task_body();

  // ---- Power management ----
  class PanelPowerGuard {
  public:
    PanelPowerGuard(EPD_Painter& d) : disp(d) {
      initOnce(d);
      xSemaphoreTake(power_mtx, portMAX_DELAY);
      if (state == 0) d.powerOn();
      state = 5;
      xSemaphoreGive(power_mtx);
    }

  private:
    EPD_Painter& disp;

    static inline TaskHandle_t      task      = nullptr;
    static inline EPD_Painter*      owner     = nullptr;
    static inline SemaphoreHandle_t power_mtx = nullptr;
    static inline uint8_t           state     = 0;

    static void initOnce(EPD_Painter& d) {
      struct Init {
        Init(EPD_Painter& d) {
          power_mtx = xSemaphoreCreateMutex();
          owner = &d;
          xTaskCreate(taskEntry, "panel_idle_off", 2048, nullptr, 1, &task);
        }
      };
      static Init init{ d };
    }

    static void taskEntry(void*) {
      for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        xSemaphoreTake(power_mtx, portMAX_DELAY);
        if (state > 0) {
          state--;
          if (state == 0) owner->powerOff();
        }
        xSemaphoreGive(power_mtx);
      }
    }
  };
};

#include "EPD_Painter_presets.h"

#endif
