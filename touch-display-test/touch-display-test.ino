/*
  touch-display-test.ino  –  CanSat Finder GND display (XIAO ESP32S3 Plus)
  ==========================================================================
  Passzív LoRa GND kijelző. A HT-CT62 (loraAPI.ino) Serial1-en (UART) tükrözi
  az összes OK:/ERR:/INFO: sort -> ezt itt parseoljuk és a TFT-re rajzoljuk.
  Nincs parancsküldés, csak megjelenítés (heltecGND.ino display-only portja).

  UART kötés (XIAO <-> HT-CT62):
    XIAO GPIO41 (TX) -> HT-CT62 GPIO20 (RX)
    XIAO GPIO42 (RX) <- HT-CT62 GPIO21 (TX)

  Panel: ILI9488 480x320 + XPT2046 touch + SD (kalibráció, változatlan)
*/

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

// ---- LoRa UART (HT-CT62 felé) --------------------------------------------
constexpr uint8_t LORA_UART_RX = 41;  // <- HT-CT62 GPIO21 (TX)
constexpr uint8_t LORA_UART_TX = 42;  // -> HT-CT62 GPIO20 (RX)
#define LoraSerial Serial1

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

// ===========================================================================
// GndDisplay – heltecGND.ino / GndDisplay.cpp portja LovyanGFX-re
// ===========================================================================

#define DISPLAY_EVENT_TTL_MS 5000

struct LoRaConfigDisp {
  float freq = 0;
  float bw = 0;
  int sf = 0;
  int power = 0;
};

class GndDisplay {
public:
  static void init() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(3);
    tft.setCursor(60, 100);
    tft.println("S.E.A.T. GND");
    tft.setTextSize(2);
    tft.setCursor(140, 150);
    tft.println("Waiting for link...");
    delay(1500);
    tft.fillScreen(TFT_BLACK);
    _drawHeader();
  }

  static void showStatus(const LoRaConfigDisp& cfg, const String& mode) {
    _cfg = cfg;
    _mode = mode;
    _drawHeader();
  }

  static void showCmd(const String& cmd) {
    tft.fillRect(0, ROW_CMD_Y, 480, ROW_H, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(4, ROW_CMD_Y + 6);
    tft.print("> " + _truncate(cmd, 38));
    _cmdTime = millis();
  }

  static void showTxStart(const String& payload) {
    tft.fillRect(0, ROW_TX_Y, 480, ROW_H, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(4, ROW_TX_Y + 6);
    tft.print("TX> " + _truncate(payload, 36));
    _txTime = millis();
  }

  static void showTxDone(uint32_t airtimeMs) {
    tft.fillRect(0, ROW_TX_Y, 480, ROW_H, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(4, ROW_TX_Y + 6);
    tft.print("TX OK  " + String(airtimeMs) + "ms");
    _txTime = millis();
  }

  static void showTxFail(const String& reason) {
    tft.fillRect(0, ROW_TX_Y, 480, ROW_H, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(4, ROW_TX_Y + 6);
    tft.print("TX FAIL " + _truncate(reason, 30));
    _txTime = millis();
  }

  static void showRx(float rssi, float snr, const String& data) {
    // Kiba**szuk a serialra a teljes adatot
    Serial.println("RX_FULL_PAYLOAD: " + data);

    tft.fillRect(0, ROW_RX1_Y, 480, ROW_H * 2, TFT_BLACK);

    String type = "";
    String lat = "";
    String lon = "";
    int tokenIndex = 0;
    int startIndex = 0;

    for (unsigned int i = 0; i <= data.length(); i++) {
      if (i == data.length() || data.charAt(i) == ';') {
        String token = data.substring(startIndex, i);
        if (tokenIndex == 0) type = token;
        else if (tokenIndex == 3) lat = token;
        else if (tokenIndex == 4) lon = token;
        startIndex = i + 1;
        tokenIndex++;
      }
    }

    tft.setTextSize(2);
    
    // Csak 2-es típusú (Avionics/GPS) csomag esetén rajzoljuk ki a lat/lon-t
    if (type.toInt() == 2) {
      tft.setTextColor(TFT_GREEN);
      tft.setCursor(4, ROW_RX1_Y + 6);
      tft.print("LAT: " + lat);
      tft.setCursor(4, ROW_RX2_Y + 6);
      tft.print("LON: " + lon);
    } else {
      tft.setTextColor(TFT_DARKGREY);
      tft.setCursor(4, ROW_RX1_Y + 6);
      tft.print("Nem GPS adat. Type: " + type);
    }

    _rxTime = millis();
  }

  static void showLinkState(bool linked) {
    if (_linked == linked) return;
    _linked = linked;
    _drawHeader();
  }

  static void update() {
    const uint32_t now = millis();

    if (_cmdTime > 0 && now - _cmdTime > DISPLAY_EVENT_TTL_MS) {
      tft.fillRect(0, ROW_CMD_Y, 480, ROW_H, TFT_BLACK);
      _cmdTime = 0;
    }
    if (_txTime > 0 && now - _txTime > DISPLAY_EVENT_TTL_MS) {
      tft.fillRect(0, ROW_TX_Y, 480, ROW_H, TFT_BLACK);
      _txTime = 0;
    }
    if (_rxTime > 0 && now - _rxTime > DISPLAY_EVENT_TTL_MS) {
      tft.fillRect(0, ROW_RX1_Y, 480, ROW_H * 2, TFT_BLACK);
      _rxTime = 0;
    }
  }

private:
  static constexpr int ROW_H = 40;
  static constexpr int ROW0_Y = 0;
  static constexpr int ROW1_Y = 40;
  static constexpr int SEP_Y = 80;
  static constexpr int ROW_CMD_Y = 96;
  static constexpr int ROW_TX_Y = 140;
  static constexpr int ROW_RX1_Y = 184;
  static constexpr int ROW_RX2_Y = 224;

  static inline LoRaConfigDisp _cfg;
  static inline String _mode = "RXTX";
  static inline bool _linked = false;
  static inline uint32_t _cmdTime = 0;
  static inline uint32_t _txTime = 0;
  static inline uint32_t _rxTime = 0;

  static String _truncate(const String& s, int maxLen) {
    if ((int)s.length() <= maxLen) return s;
    return s.substring(0, maxLen - 1) + "~";
  }

  static void _drawHeader() {
    tft.fillRect(0, 0, 480, SEP_Y, TFT_BLACK);

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(4, ROW0_Y + 6);
    tft.print("S.E.A.T. GND");

    uint16_t badgeColor = _linked ? TFT_GREEN : TFT_RED;
    String badgeText = "[" + _mode + "]";
    tft.setTextColor(badgeColor);
    int badgeW = badgeText.length() * 12;
    tft.setCursor(476 - badgeW, ROW0_Y + 6);
    tft.print(badgeText);

    char buf[48];
    snprintf(buf, sizeof(buf), "%.1fMHz  SF%d  BW%.0f  P%ddBm",
             _cfg.freq, _cfg.sf, _cfg.bw, _cfg.power);
    tft.setTextColor(TFT_SKYBLUE);
    tft.setCursor(4, ROW1_Y + 6);
    tft.print(buf);

    tft.drawFastHLine(0, SEP_Y - 1, 480, TFT_DARKGREY);
  }
};

// ===========================================================================
// LoRa serial line parser
// ===========================================================================

class LoraLineParser {
public:
  static void begin() {
    LoraSerial.begin(115200, SERIAL_8N1, LORA_UART_RX, LORA_UART_TX);
  }

  static void update() {
    while (LoraSerial.available()) {
      char c = LoraSerial.read();
      if (c == '\n') {
        _line.trim();
        if (_line.length() > 0) _handleLine(_line);
        _line = "";
      } else if (c != '\r') {
        _line += c;
        if (_line.length() > 400) _line = ""; 
      }
    }

    if (GndDisplay_linked && millis() - _lastRxMs > 8000) {
      GndDisplay_linked = false;
      GndDisplay::showLinkState(false);
    }
  }

private:
  static inline String _line;
  static inline uint32_t _lastRxMs = 0;
  static inline bool GndDisplay_linked = false;

  static void _handleLine(const String& line) {
    Serial.println(line);
    _lastRxMs = millis();
    if (!GndDisplay_linked) {
      GndDisplay_linked = true;
      GndDisplay::showLinkState(true);
    }

    if (line.startsWith("INFO: RX_DATA")) {
      _handleRxData(line);
    } else if (line.startsWith("INFO: TX_START")) {
      _handleTxStart(line);
    } else if (line.startsWith("INFO: TX_DONE")) {
      _handleTxDone(line);
    } else if (line.startsWith("INFO: TIMEOUT")) {
      GndDisplay::showTxFail("TX_TIMEOUT");
    } else if (line.startsWith("INFO: RX_CMD")) {
      GndDisplay::showCmd(line.substring(14));
    } else if (line.startsWith("OK: freq:")) {
      _handleConfigLine(line);
    } else if (line.startsWith("OK: send")) {
      GndDisplay::showCmd(line);
    } else if (line.startsWith("ERR:") || line.startsWith("INVALID:")) {
      GndDisplay::showCmd(line);
    }
  }

  static void _handleRxData(const String& line) {
    int p1 = line.indexOf("RSSI:");
    int p2 = line.indexOf("SNR:");
    int p3 = line.indexOf('|', p2);
    if (p1 == -1 || p2 == -1 || p3 == -1) return;

    float rssi = line.substring(p1 + 5, line.indexOf('|', p1)).toFloat();
    float snr = line.substring(p2 + 4, p3).toFloat();
    String data = line.substring(p3 + 1);
    data.trim();

    GndDisplay::showRx(rssi, snr, data);
  }

  static void _handleTxStart(const String& line) {
    int p = line.indexOf("Len:");
    String lenStr = (p == -1) ? "" : line.substring(p + 4);
    lenStr.trim();
    GndDisplay::showTxStart("Len:" + lenStr);
  }

  static void _handleTxDone(const String& line) {
    int p = line.indexOf("Airtime:");
    if (p == -1) {
      GndDisplay::showTxDone(0);
      return;
    }
    String msStr = line.substring(p + 8);
    msStr.trim();
    msStr.replace("ms", "");
    msStr.trim();
    GndDisplay::showTxDone((uint32_t)msStr.toInt());
  }

  static void _handleConfigLine(const String& line) {
    LoRaConfigDisp cfg;
    cfg.freq = _extractFloat(line, "freq:");
    cfg.bw = _extractFloat(line, "bw:");
    cfg.sf = (int)_extractFloat(line, "sf:");
    cfg.power = (int)_extractFloat(line, "power:");
    GndDisplay::showStatus(cfg, "RXTX");
  }

  static float _extractFloat(const String& line, const String& key) {
    int p = line.indexOf(key);
    if (p == -1) return 0;
    p += key.length();
    int end = line.indexOf(';', p);
    if (end == -1) end = line.length();
    return line.substring(p, end).toFloat();
  }
};

// ===========================================================================
// setup / loop
// ===========================================================================

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

  LoraLineParser::begin();
  GndDisplay::init();
}

void loop() {
  LoraLineParser::update();
  GndDisplay::update();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      LoraSerial.println(cmd);
    }
  }

  int32_t x, y;
  if (tft.getTouch(&x, &y)) {
    tft.fillCircle(x, y, 3, TFT_BLUE);
  }
}