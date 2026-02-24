#ifndef EPD_Painter_H
#define EPD_Painter_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>


class EPD_Painter : public GFXcanvas8 {

public:
  struct Config {
      int8_t pin_pwr;
      int8_t pin_sph;
      int8_t pin_oe;
      int8_t pin_cl;
      int8_t pin_spv;
      int8_t pin_ckv;
      int8_t pin_le;
      int8_t data_pins[8];
      uint16_t width;
      uint16_t height;
      int8_t latch_delay=10;
  };

  enum class Quality {
    QUALITY_HIGH,
    QUALITY_NORMAL,
    QUALITY_FAST
  };


  EPD_Painter(const Config &config);
  bool begin();
  bool end();

  void clear();
  void paint();

  void setQuality(Quality quality);

  Config _config;

private:



  // ---- LCD_CAM / DMA ----
  gdma_channel_handle_t dma_chan = nullptr;
  dma_descriptor_t      dma_desc1 = {};
  dma_descriptor_t      dma_desc2 = {};

  // ---- Buffers ----
  uint8_t* dma_buffer        = nullptr;  // Points at one of the buffers below
  uint8_t* dma_buffer1       = nullptr;  //  Row Double buffer A
  uint8_t* dma_buffer2       = nullptr;  //. Row Double buffer B

  uint8_t* packed_fastbuffer  = nullptr;  // 2bpp current frame  (internal RAM)
  uint8_t* packed_screenbuffer = nullptr; // 2bpp previous frame (PSRAM)

  

  int packed_row_bytes = 0;
  bool interlace_period = false;

  // ---- Internal helpers ----
  void powerOn();
  void powerOff();
  void sendRow(bool firstLine, bool lastLine=false);

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