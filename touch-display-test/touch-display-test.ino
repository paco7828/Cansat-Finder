#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SPI.h>
#include <SD.h>

#define SD_CLK 39
#define SD_MISO 40
#define SD_MOSI 41
#define SD_CS 42

#define DISPLAY_CLK 12
#define DISPLAY_MOSI 11
#define DISPLAY_MISO 13
#define DISPLAY_DC 9
#define DISPLAY_CS 10
#define DISPLAY_RESET 8

#define TOUCH_CLK 12
#define TOUCH_MOSI 11
#define TOUCH_MISO 13
#define TOUCH_CS 7
#define TOUCH_IRQ 6

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 27000000;
      cfg.freq_read = 16000000;
      cfg.pin_sclk = DISPLAY_CLK;
      cfg.pin_mosi = DISPLAY_MOSI;
      cfg.pin_miso = DISPLAY_MISO;
      cfg.pin_dc = DISPLAY_DC;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = DISPLAY_CS;
      cfg.pin_rst = DISPLAY_RESET;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _touch_instance.config();

      cfg.x_min = 250;
      cfg.x_max = 3750;

      cfg.y_min = 3600;
      cfg.y_max = 500;

      cfg.pin_sclk = TOUCH_CLK;
      cfg.pin_mosi = TOUCH_MOSI;
      cfg.pin_miso = TOUCH_MISO;
      cfg.pin_cs = TOUCH_CS;

      cfg.pin_int = TOUCH_IRQ;

      cfg.bus_shared = true;
      cfg.spi_host = SPI2_HOST;
      cfg.freq = 1000000;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
SPIClass spiSD(FSPI);

void setup() {
  Serial.begin(115200);
  delay(500);

  spiSD.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("No SD");
  } else {
    Serial.println("SD OK");
  }

  pinMode(DISPLAY_CS, OUTPUT);
  digitalWrite(DISPLAY_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TOUCH_IRQ, INPUT_PULLUP);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  uint16_t calData[8];
  tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 20);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Calibration done", 10, 10);
}

void loop() {
  int32_t x, y;
  if (tft.getTouch(&x, &y)) {
    tft.fillCircle(x, y, 5, TFT_RED);
    Serial.printf("X=%d Y=%d\n", x, y);
  }
}