#ifndef EPD_Painter_H
#define EPD_Painter_H

#include <stddef.h>

#if !defined(EPD_PAINTER_PRESET_M5PAPER_S3) && \
    !defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) && \
    !defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752)
  
  #ifndef EPD_PAINTER_PRESET_AUTO
    #define EPD_PAINTER_PRESET_AUTO
  #endif
#endif

// Forward declaration — avoids circular include with epd_painter_shutdown.h
class EPD_PainterShutdown;

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
    ROTATION_CW   // 90° clockwise — portrait drawing canvas (width↔height swapped)
  };

  struct Waveforms {
      uint8_t fast_lighter[3][7];
      uint8_t fast_darker[3][7];
      uint8_t normal_lighter[3][13];
      uint8_t normal_darker[3][13];
      uint8_t high_lighter[3][13];
      uint8_t high_darker[3][13];
  };

  struct Shift {
    int8_t data   = -1;
    int8_t clk    = -1; 
    int8_t strobe = -1;
    int8_t le_time = 0;
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
      int8_t data_pins[8];
      I2CBusConfig i2c{};
      PowerCtlConfig power{};
      Waveforms waveforms;
      Shift shift;

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

  void fxClear();
  void clearBuffers();  // zero all packed buffers (call before power-off to reset DC-balance baseline)
  void paint(uint8_t* framebuffer);
  void paintPacked(const uint8_t* packed);
  void unpaintPacked(const uint8_t* packed);
  void paintLater(uint8_t* framebuffer);
  void setInterlaceMode(bool mode){
    interlace_mode = mode;
  }

  void setQuality(Quality quality);

  // Dither an 8bpp framebuffer (0=black … 255=white) in-place to the driver's
  // 4-level encoding: 0=white, 1=lt grey, 2=dk grey, 3=black.
  // Uses Floyd-Steinberg error diffusion. Allocates one row of int16 scratch
  // (~2 KB for a 960-wide panel) internally.
  static void dither(uint8_t* fb, uint16_t width, uint16_t height);

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

  uint8_t* packed_fastbuffer  = nullptr;  // 2bpp current frame  (internal RAM)
  uint8_t* packed_screenbuffer = nullptr; // 2bpp previous frame (PSRAM)
  uint8_t* packed_paintbuffer = nullptr; // 2bpp previous frame (PSRAM)

  uint32_t* bitmask = nullptr;

  int packed_row_bytes = 0;
  std::atomic<int> paintStage{0};
  bool interlace_mode = false;
  bool shouldSkipRow = false;
  bool _autoShutdown = true;
  EPD_PainterShutdown* _shutdown = nullptr;


  // ---- Internal helpers ----
  void powerOn();
  void powerOff();
  bool autoDetectBoard();
  void sendRow(bool firstLine, bool lastLine=false, bool skipRow=false);

  void shiftOn(int bitmask);
  void shiftOff(int bitmask);

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
