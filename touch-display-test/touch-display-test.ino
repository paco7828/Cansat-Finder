#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SPI.h>
#include <SD.h>

constexpr uint8_t TFT_CS = 2;
constexpr uint8_t TFT_RST = 3;
constexpr uint8_t TFT_DC = 4;
constexpr uint8_t TFT_MOSI = 9;
constexpr uint8_t TFT_MISO = 8;
constexpr uint8_t TFT_CLK = 7;
constexpr uint8_t TOUCH_CS = 5;
constexpr uint8_t TOUCH_IRQ = 6;
constexpr uint8_t SD_CS = 38;
constexpr uint8_t SD_MOSI = 11;
constexpr uint8_t SD_MISO = 12;
constexpr uint8_t SD_CLK = 13;

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
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.pin_sclk = TFT_CLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc = TFT_DC;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = TFT_CS;
      cfg.pin_rst = TFT_RST;
      cfg.panel_width = 320;
      cfg.panel_height = 480;
      cfg.readable = false;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 250;
      cfg.x_max = 3750;
      cfg.y_min = 500;
      cfg.y_max = 3600;
      cfg.pin_sclk = TFT_CLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = TFT_MISO;
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
SPIClass spiSD(HSPI);

void setup() {
  Serial.begin(115200);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);

  spiSD.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  tft.setCursor(10, 10);
  bool sdOK = SD.begin(SD_CS, spiSD, 4000000);

  if (!sdOK) {
    Serial.println("SD FAIL");
    tft.setTextColor(TFT_RED);
    tft.println("SD: FAIL");
  } else {
    Serial.println("SD OK");
    tft.setTextColor(TFT_GREEN);
    tft.println("SD: OK");
  }

  uint16_t calData[8];
  if (sdOK && SD.exists("/touch_cal.txt")) {
    File f = SD.open("/touch_cal.txt", "r");
    if (f) {
      f.read((uint8_t*)calData, 16);
      f.close();
      tft.setTouchCalibrate(calData);
      Serial.println("Kalibracio betoltve SD-rol.");
    }
  } else {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Nyomd meg a sarkokat a kalibraciohoz!");

    tft.calibrateTouch(calData, TFT_WHITE, TFT_RED, 15);

    if (sdOK) {
      File f = SD.open("/touch_cal.txt", "w");
      if (f) {
        f.write((uint8_t*)calData, 16);
        f.close();
        Serial.println("Uj kalibracio elmentve az SD-re.");
      }
    }
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(3);
  }
}

void loop() {
  int32_t x, y;
  if (tft.getTouch(&x, &y)) {
    tft.fillCircle(x, y, 3, TFT_BLUE);
  }
}