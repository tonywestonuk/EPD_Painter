#ifndef EPD_Painter_H
#define EPD_Painter_H

#include <Arduino.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include "Wire.h"


class EPD_Painter {

public:

struct I2CBusConfig {
    TwoWire* wire = nullptr;
    int sda = -1;
    int scl = -1;
    uint32_t freq = 100000;
};
struct PowerCtlConfig {
    int pca_addr=-1;
    int tps_addr=-1;
    int vcom_mv=-1;
};
  enum class Quality {
    QUALITY_HIGH,
    QUALITY_NORMAL,
    QUALITY_FAST
  };

  struct Config {
      uint16_t width;
      uint16_t height;
      int8_t pin_pwr;
      int8_t pin_sph;
      int8_t pin_oe;
      int8_t pin_cl;
      int8_t pin_spv;
      int8_t pin_ckv;
      int8_t pin_le;
      int8_t latch_delay;
      Quality quality;
      int8_t data_pins[8];
      I2CBusConfig i2c{};
      PowerCtlConfig power{};
  };


  Config _config;

  EPD_Painter(const Config &config);
  bool begin();
  bool end();

  void clear();
  void paint(uint8_t* framebuffer);

  void setQuality(Quality quality);

  Config getConfig(){
    return _config;
  }


private:



  // ---- LCD_CAM / DMA ----
  gdma_channel_handle_t dma_chan = nullptr;
  dma_descriptor_t      dma_desc1 = {};
  dma_descriptor_t      dma_desc2 = {};

  // ---- Buffers ----
  
  uint8_t* dma_buffer        = nullptr;  // Points at one of the buffers below
  uint8_t* dma_buffer1       = nullptr;  //  Row Double buffer A
  uint8_t* dma_buffer2       = nullptr;  //. Row Double buffer B

  uint8_t* packed_fastbuffer   = nullptr;   // 2bpp working buffer for draw passes
  uint8_t* _pack_bufs[2]       = {};        // 2bpp staging queue buffers
  uint8_t* _pqueue[2]          = {};        // ring buffer of pointers into _pack_bufs
  int      _pq_head            = 0;
  int      _pq_count           = 0;
  int      _pq_wbuf            = 0;         // which _pack_bufs slot to write to next
  SemaphoreHandle_t _pq_mutex  = nullptr;
  SemaphoreHandle_t _pq_sem    = nullptr;   // counting semaphore, max 2
  uint8_t* packed_screenbuffer = nullptr;   // 2bpp previous frame (PSRAM)
  uint32_t* bitmask            = nullptr;

  

  int packed_row_bytes = 0;
  bool interlace_period = false;
  bool shouldSkipRow = false;

  // ---- Internal helpers ----
  void powerOn();
  void powerOff();
  void sendRow(bool firstLine, bool lastLine=false, bool skipRow=false);
  void _compactFrame(uint8_t* framebuffer, uint8_t* dest);
  void _drawPasses();
  static void _drawTaskEntry(void* arg);
  TaskHandle_t _draw_task = nullptr;

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

    static inline TaskHandle_t     task      = nullptr;
    static inline EPD_Painter*     owner     = nullptr;
    static inline SemaphoreHandle_t power_mtx = nullptr;
    static inline uint8_t          state     = 0;

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

#endif