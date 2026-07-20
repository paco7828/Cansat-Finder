#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Preferences.h>
#include "config.h"
#include "CS26_TelemetryData.h"
#include "CS26_TelemetryParser.h"
#include "Better-GPS.h"
#include "Better-Capportal.h"
#include "captive-portal-web.h"
#include "Lopaka-UI.h"
#include <qrcode.h>

// Lovyan GFX colors
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_YELLOW 0xFFE0

constexpr int SKIP_BTN_W = 110;
constexpr int SKIP_BTN_H = 36;
constexpr int SKIP_BTN_X = 382 - SKIP_BTN_W / 2;
constexpr int SKIP_BTN_Y = 275;

// Global variables
AppState appState = STATE_TITLE;
unsigned long lastDisplayUpdate = 0;
unsigned long titleStartTime = 0;
unsigned long lastAnimationUpdate = 0;
unsigned long lastBeepTime = 0;
unsigned long beepInterval = 0;
bool beepActive = false;
unsigned long beepStartTime = 0;
int beepDuration = 0;
int beepFrequency = 0;
bool sdCardAvailable = false;
unsigned long lastSDWrite = 0;
String currentLogFile = "";
unsigned long apStartTime = 0;
String cansatName = DEFAULT_CANSAT_NAME;
int lastClientCount = -1;

// Enums
GPSData gpsData;
CanSatData canSatData;
PreviousValues prevVals;
LoRaConfig loraConfig;
TelemetryData gndStore;

// LovyanGFX Driver config
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void)
  {
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

// Instances
LGFX tft;
SPIClass spiSD(HSPI);
BetterGPS gps;
BetterCapportal capportal;
Preferences preferences;

// Function prototypes
void playStartupMelody();
void beep(int frequency, int duration);
void updateBeepInterval(double distance);
void parseLoRaData(String data);
void drawTitleScreen();
void drawConfigScreen();
void drawWifiQR();
void drawAnimation_gears_64_64_28f();
void drawAnimation_activity_64_64_28f();
void drawRunningScreen();
void updateDisplay();
void updateValueArea(int x, int y, int w, int h, String value, uint16_t color);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2);
void drawArrow(int cx, int cy, int radius, double bearing);
void handleConfigSubmission(std::map<String, String> &data);
void enterConfigMode();
void loadConfig();
void saveConfig();
void initializeSDCard();
int countSDFiles(const char *dirname);
String buildTelemetryString();
void writeToSD();
void drawSolarPanels(int cx, int cy, int radius);
void readLoRaSerial();

void setup()
{
  Serial.begin(115200);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(BUZZER, OUTPUT);

  // Load config
  preferences.begin("cansat", false);
  loadConfig();

  // LovyanGFX init
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // SD init
  initializeSDCard();

  // Load touch calibration file from SD
  uint16_t calData[8];
  bool calLoaded = false;

  if (sdCardAvailable && SD.exists("/touch_cal.txt"))
  {
    File f = SD.open("/touch_cal.txt", "r");
    if (f)
    {
      size_t bytesRead = f.read((uint8_t *)calData, sizeof(calData));
      f.close();
      if (bytesRead == sizeof(calData))
      {
        tft.setTouchCalibrate(calData);
        calLoaded = true;
        Serial.println("Touch calib loaded");
      }
    }
  }

  if (!calLoaded)
  {
    Serial.println("Touch calibration file not found");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Nyomd meg a sarkokat!");

    tft.calibrateTouch(calData, TFT_WHITE, TFT_RED, 15);

    if (sdCardAvailable)
    {
      File f = SD.open("/touch_cal.txt", "w");
      if (f)
      {
        f.write((uint8_t *)calData, sizeof(calData));
        f.close();
        Serial.println("✅ Új touch kalibráció elmentve SD-re.");
      }
    }
    tft.fillScreen(TFT_BLACK);
  }

  titleStartTime = millis();
  drawTitleScreen();
  playStartupMelody();

  // UART for HT-CT62
  Serial1.begin(115200, SERIAL_8N1, LORA_UART_RX, LORA_UART_TX);
}

void loop()
{
  unsigned long currentTime = millis();

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0)
    {
      Serial1.println(cmd);
    }
  }

  // Buzzer stop logic
  if (beepActive && (millis() - beepStartTime >= beepDuration))
  {
    noTone(BUZZER);
    beepActive = false;
  }

  switch (appState)
  {
  case STATE_TITLE:
    if (currentTime - titleStartTime >= TITLE_DURATION)
    {
      appState = STATE_CONFIG;
      enterConfigMode();
    }
    break;

  case STATE_CONFIG:
  {
    capportal.handle();

    // Listen for touch
    int32_t tx, ty;
    if (tft.getTouch(&tx, &ty))
    {
      Serial.println("Touch X: " + String(tx) + " Y: " + String(ty));

      if (tx >= SKIP_BTN_X && tx <= SKIP_BTN_X + SKIP_BTN_W && ty >= SKIP_BTN_Y && ty <= SKIP_BTN_Y + SKIP_BTN_H)
      {
        capportal.stop();
        delay(100);
        appState = STATE_INITIALIZING;
        break;
      }
    }

    if (millis() - apStartTime >= AP_TIMEOUT)
    {
      capportal.stop();
      delay(100);
      appState = STATE_INITIALIZING;
      break;
    }

    if (millis() - lastAnimationUpdate >= ANIMATION_UPDATE_INTERVAL)
    {
      drawAnimation_gears_64_64_28f();
      lastAnimationUpdate = millis();
    }

    int currentClients = capportal.clientCount();
    if (currentClients != lastClientCount)
    {
      if (currentClients > lastClientCount && lastClientCount >= 0)
      {
        beep(3700, 100);
      }

      updateValueArea(174, 221, 60, 18, String(currentClients), TFT_GREEN);
      lastClientCount = currentClients;
    }
    break;
  }

  case STATE_INITIALIZING:
    gps.begin(GPS_RX, GPS_TX, 9600);

    // Format (HeltecCmdInterpreter::parseSetConfig): freq,bw,sf,cr,sw,power,pl,tcxo,crc
    {
      float freqMHz = loraConfig.frequency.toFloat() / 1000000.0;
      uint8_t syncByte = (uint8_t)strtol(loraConfig.sync.c_str(), nullptr, 16);
      String cfgCmd = "set_config " + String(freqMHz, 1) + "," + loraConfig.bandwidth + ",7,5," + String(syncByte) + ",14,8,1.6,2";
      Serial1.println(cfgCmd);
      Serial.println("LORA_TX: " + cfgCmd);
      delay(150);
    }

    appState = STATE_RUNNING;
    drawRunningScreen();
    break;

  case STATE_RUNNING:
    gps.update();
    readLoRaSerial();

    if (canSatData.valid && (millis() - canSatData.lastReceived > LORA_TIMEOUT))
    {
      canSatData.valid = false;
    }

    if (gps.hasFix())
    {
      gpsData.hasFix = true;
      gpsData.latitude = gps.getLatitude();
      gpsData.longitude = gps.getLongitude();
      gpsData.altitude = gps.getAltitude();
      gpsData.speed = gps.getSpeedKmph() / 3.6;
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
      if (gps.hasCourse())
      {
        gpsData.course = gps.getCourse();
      }
    }
    else
    {
      gpsData.hasFix = false;
      gpsData.latitude = 0.0;
      gpsData.longitude = 0.0;
      gpsData.altitude = 0.0;
      gpsData.speed = 0.0;
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
    }

    if (gpsData.hasFix && canSatData.valid && !beepActive)
    {
      double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
      updateBeepInterval(distance);

      if (beepInterval > 0 && (currentTime - lastBeepTime >= beepInterval))
      {
        int currentDuration = constrain((int)(beepInterval / 2), 25, 150);
        beep(beepFrequency, currentDuration);
        lastBeepTime = currentTime;
      }
    }
    else
    {
      beepInterval = 0;
    }

    if (millis() - lastAnimationUpdate >= ANIMATION_UPDATE_INTERVAL)
    {
      drawAnimation_activity_64_64_28f();
      lastAnimationUpdate = millis();
    }

    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }

    if (sdCardAvailable && (currentTime - lastSDWrite >= SD_WRITE_INTERVAL))
    {
      writeToSD();
      lastSDWrite = currentTime;
    }
    break;
  }
  delay(10);
}

void playStartupMelody()
{
  int notes[] = {440, 523, 587, 659, 880, 880, 880};
  int durations[] = {100, 250, 250, 600, 50, 50, 50};
  int gaps[] = {40, 40, 40, 150, 30, 30, 0};
  for (int i = 0; i < 7; i++)
  {
    tone(BUZZER, notes[i]);
    delay(durations[i]);
    noTone(BUZZER);
    delay(gaps[i]);
  }
}

void beep(int frequency, int duration)
{
  if (beepActive)
  {
    noTone(BUZZER);
    beepActive = false;
  }
  beepStartTime = millis();
  beepDuration = duration;
  beepFrequency = frequency;
  tone(BUZZER, frequency);
  beepActive = true;
}

void updateBeepInterval(double distance)
{
  double clamped = constrain(distance, MIN_DISTANCE, MAX_DISTANCE);
  double ratio = (clamped - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
  double curve = sqrt(ratio);

  beepInterval = MIN_BEEP_INTERVAL + curve * (MAX_BEEP_INTERVAL - MIN_BEEP_INTERVAL);
  beepFrequency = map((long)(ratio * 1000), 0, 1000, 4200, 2200);
}

void parseLoRaData(String data)
{
  int lastPipe = data.lastIndexOf('|');
  String csv = (lastPipe == -1) ? data : data.substring(lastPipe + 1);
  csv.trim();

  uint8_t type = CS26Telem::detectType(csv);

  if (type == 2) // Avionics/GPS -> canSatData
  {
    if (CS26Telem::parseAvionics(csv, gndStore))
    {
      if (gndStore.gps_valid)
      {
        canSatData.latitude = gndStore.gps_lat;
        canSatData.longitude = gndStore.gps_lon;
        canSatData.altitude = gndStore.gps_alt_m;
        canSatData.speed = gndStore.gps_speed;
        canSatData.valid = true;
        canSatData.lastReceived = millis();

        char timeStr[9];
        sprintf(timeStr, "%02u:%02u:%02u", gndStore.gps_hour, gndStore.gps_min, gndStore.gps_sec);
        canSatData.lastReceivedTimeStr = String(timeStr);
      }
    }
  }
}

void drawTitleScreen()
{
  tft.fillScreen(0x0);
  tft.setTextColor(0x4FE0);
  tft.setTextSize(5);
  tft.drawString("CanSat Finder", 53, 20);
  tft.drawBitmap(-181, 30, image_seatlogo_bits, 819, 320, 0xFFFF);
}

void drawConfigScreen()
{
  tft.fillScreen(0x0);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(4);
  tft.drawString("Configuration", 16, 20);

  tft.drawBitmap(20, 75, image_wifi_bits, 38, 30, 0xFFFF);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("SSID:", 70, 86);
  tft.drawString(String(SSID), 138, 86);

  tft.drawBitmap(20, 120, image_device_key_bits, 30, 30, 0xFFFF);
  tft.drawString("PASS:", 70, 126);
  tft.drawString(String(PASSWRD), 138, 126);

  tft.drawBitmap(20, 165, image_network_www_bits, 32, 32, 0xFFFF);
  tft.drawString("IP:", 70, 173);
  tft.drawString(capportal.getIP(), 114, 173);

  tft.drawBitmap(20, 210, image_phone_contacts_bits, 26, 32, 0xFFFF);
  tft.drawString("CLIENTS:", 70, 221);
  updateValueArea(174, 221, 60, 18, String(capportal.clientCount()), TFT_GREEN);
  lastClientCount = capportal.clientCount();

  if (sdCardAvailable)
    tft.drawBitmap(20, 255, image_micro_sd_bits, 24, 30, 0xFFFF);
  else
    tft.drawBitmap(18, 253, image_micro_sd_no_card_bits, 28, 32, 0xFFFF);
  tft.drawString("SD:", 70, 265);
  tft.setTextColor(sdCardAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.drawString(sdCardAvailable ? "OK" : "FAIL", 114, 265);

  drawWifiQR();

  // SKIP button
  tft.fillRoundRect(SKIP_BTN_X, SKIP_BTN_Y, SKIP_BTN_W, SKIP_BTN_H, 6, TFT_BLACK);
  tft.drawRoundRect(SKIP_BTN_X, SKIP_BTN_Y, SKIP_BTN_W, SKIP_BTN_H, 6, TFT_WHITE);
  tft.drawRoundRect(SKIP_BTN_X + 1, SKIP_BTN_Y + 1, SKIP_BTN_W - 2, SKIP_BTN_H - 2, 5, TFT_WHITE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(middle_center);
  tft.drawString("SKIP", SKIP_BTN_X + SKIP_BTN_W / 2, SKIP_BTN_Y + SKIP_BTN_H / 2);
  tft.setTextDatum(top_left);
}

void drawWifiQR()
{
  String wifiURI = "WIFI:T:WPA;S:" + String(SSID) + ";P:" + String(PASSWRD) + ";H:false;;";

  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(4)];
  qrcode_initText(&qrcode, qrcodeData, 4, ECC_MEDIUM, wifiURI.c_str());

  const int scale = 4;
  const int quiet = 4;
  const int boxX = 300;
  const int boxY = 100;
  const int boxSize = (qrcode.size + quiet * 2) * scale;

  tft.fillRect(boxX, boxY, boxSize, boxSize, TFT_WHITE);

  int qrX = boxX + quiet * scale;
  int qrY = boxY + quiet * scale;

  for (uint8_t y = 0; y < qrcode.size; y++)
  {
    for (uint8_t x = 0; x < qrcode.size; x++)
    {
      if (qrcode_getModule(&qrcode, x, y))
      {
        tft.fillRect(qrX + x * scale, qrY + y * scale, scale, scale, TFT_BLACK);
      }
    }
  }
}

void drawAnimation_gears_64_64_28f()
{
  gears_64_64_28f_frame = (millis() / 42) % 28;
  tft.drawBitmap(346, 5, gears_64_64_28f_frames[gears_64_64_28f_frame], 64, 64, TFT_WHITE, TFT_BLACK);
}

void drawAnimation_activity_64_64_28f()
{
  activity_64_64_28f_frame = (millis() / 42) % 28;
  tft.drawBitmap(414, 0, activity_64_64_28f_frames[activity_64_64_28f_frame], 64, 64, TFT_WHITE, TFT_BLACK);
}

void drawRunningScreen()
{
  tft.fillScreen(0x0);

  tft.drawBitmap(5, 5, image_satellite_dish_bits, 30, 30, 0xFFFF);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(3);
  tft.drawString("CanSat", 45, 9);

  if (sdCardAvailable)
    tft.drawBitmap(480 - 24, 320 - 30, image_micro_sd_bits, 24, 30, 0xFFFF);
  else
    tft.drawBitmap(480 - 28, 320 - 32, image_micro_sd_no_card_bits, 28, 32, 0xFFFF);

  tft.setTextSize(2);
  tft.setTextColor(0xFFFF, TFT_BLACK);
  tft.drawBitmap(10, 45, image_location_map_bits, 15, 16, 0xFFFF);
  tft.drawString("Lat:", 40, 46);
  tft.drawBitmap(10, 69, image_location_map_bits, 15, 16, 0xFFFF);
  tft.drawString("Lon:", 40, 70);
  tft.drawBitmap(10, 93, image_plane_bits, 16, 16, 0xFFFF);
  tft.drawString("Speed:", 40, 94);
  tft.drawBitmap(10, 117, image_arrow_up_bits, 10, 14, 0xFFFF);
  tft.drawString("Altitude:", 40, 118);
  tft.drawBitmap(10, 141, image_clock_bits, 15, 16, 0xFFFF);
  tft.drawString("Last:", 40, 142);

  tft.drawBitmap(10, 172, image_smartphone_bits, 18, 32, 0xFFFF);
  tft.setTextSize(3);
  tft.drawString("You", 45, 178);

  tft.setTextSize(2);
  tft.drawBitmap(10, 214, image_location_map_bits, 15, 16, 0xFFFF);
  tft.drawString("Lat:", 40, 215);
  tft.drawBitmap(10, 238, image_location_map_bits, 15, 16, 0xFFFF);
  tft.drawString("Lon:", 40, 239);
  tft.drawBitmap(10, 262, image_plane_bits, 16, 16, 0xFFFF);
  tft.drawString("Speed:", 40, 263);
  tft.drawBitmap(10, 286, image_arrow_up_bits, 10, 14, 0xFFFF);
  tft.drawString("Altitude:", 40, 287);

  tft.drawCircle(380, 160, 70, TFT_WHITE);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(320, 245);
  tft.println("Distance:");

  prevVals.csLat = -999;
  prevVals.csLastTime = "";
}

void updateDisplay()
{
  uint16_t csColor = canSatData.valid ? TFT_GREEN : TFT_RED;

  bool csValidChanged = (prevVals.csValid != canSatData.valid);
  prevVals.csValid = canSatData.valid;

  if (prevVals.csLat != canSatData.latitude || csValidChanged)
  {
    updateValueArea(160, 46, 130, 18, String(canSatData.latitude, 6), csColor);
    prevVals.csLat = canSatData.latitude;
  }
  if (prevVals.csLon != canSatData.longitude || csValidChanged)
  {
    updateValueArea(160, 70, 130, 18, String(canSatData.longitude, 6), csColor);
    prevVals.csLon = canSatData.longitude;
  }
  if (prevVals.csSpeed != canSatData.speed || csValidChanged)
  {
    updateValueArea(160, 94, 130, 18, String(canSatData.speed, 1) + " m/s", csColor);
    prevVals.csSpeed = canSatData.speed;
  }
  if (prevVals.csAlt != canSatData.altitude || csValidChanged)
  {
    updateValueArea(160, 118, 130, 18, String(canSatData.altitude, 1) + " m", csColor);
    prevVals.csAlt = canSatData.altitude;
  }
  {
    String csAgeStr;
    uint16_t csAgeColor;

    if (canSatData.lastReceived == 0)
    {
      csAgeStr = "--";
      csAgeColor = TFT_RED;
    }
    else
    {
      unsigned long ageMs = millis() - canSatData.lastReceived;
      csAgeStr = String(ageMs / 1000) + "s";
      csAgeColor = (ageMs <= LORA_TIMEOUT) ? TFT_GREEN : TFT_RED;
    }

    if (prevVals.csLastTime != csAgeStr)
    {
      updateValueArea(160, 142, 130, 18, csAgeStr, csAgeColor);
      prevVals.csLastTime = csAgeStr;
    }
  }

  uint16_t gpsColor = gpsData.hasFix ? TFT_GREEN : TFT_RED;

  if (prevVals.gpLat != gpsData.latitude)
  {
    updateValueArea(160, 215, 130, 18, String(gpsData.latitude, 6), gpsColor);
    prevVals.gpLat = gpsData.latitude;
  }
  if (prevVals.gpLon != gpsData.longitude)
  {
    updateValueArea(160, 239, 130, 18, String(gpsData.longitude, 6), gpsColor);
    prevVals.gpLon = gpsData.longitude;
  }
  if (prevVals.gpSpeed != gpsData.speed)
  {
    updateValueArea(160, 263, 130, 18, String(gpsData.speed, 1) + " m/s", gpsColor);
    prevVals.gpSpeed = gpsData.speed;
  }
  if (prevVals.gpAlt != gpsData.altitude)
  {
    updateValueArea(160, 287, 130, 18, String(gpsData.altitude, 1) + " m", gpsColor);
    prevVals.gpAlt = gpsData.altitude;
  }

  if (gpsData.hasFix && canSatData.valid)
  {
    double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    double absoluteBearing = calculateBearing(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);

    if (abs(prevVals.distance - distance) > 0.5)
    {
      String distStr = (distance < 1000) ? String((int)distance) + " m" : String(distance / 1000.0, 2) + " km";
      updateValueArea(300, 265, 150, 18, distStr, TFT_YELLOW);
      prevVals.distance = distance;
    }

    double bearingToDisplay = absoluteBearing;
    bool needsRedraw = false;

    if (gpsData.speed > 1.0 && gps.hasCourse())
    {
      double relativeBearing = absoluteBearing - gpsData.course;
      while (relativeBearing < 0)
        relativeBearing += 360;
      while (relativeBearing >= 360)
        relativeBearing -= 360;
      bearingToDisplay = relativeBearing;
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 5.0 || abs(prevVals.course - gpsData.course) > 5.0);
    }
    else
    {
      bearingToDisplay = absoluteBearing;
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 5.0);
    }

    if (needsRedraw)
    {
      tft.fillRect(380 - 70, 160 - 70, 140, 140, TFT_BLACK);
      drawSolarPanels(380, 160, 70);
      tft.drawCircle(380, 160, 70, TFT_WHITE);
      drawArrow(380, 160, 60, bearingToDisplay);
      prevVals.bearing = absoluteBearing;
      prevVals.course = gpsData.course;
    }
  }
  else
  {
    if (prevVals.distance != -1)
    {
      updateValueArea(300, 265, 150, 18, "---", TFT_YELLOW);
      prevVals.distance = -1;
      tft.fillRect(380 - 70, 160 - 70, 140, 140, TFT_BLACK);
      tft.drawCircle(380, 160, 70, TFT_WHITE);
      prevVals.bearing = -999;
      prevVals.course = -999;
    }
  }
}

void updateValueArea(int x, int y, int w, int h, String value, uint16_t color)
{
  tft.fillRect(x, y, w, h, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  tft.println(value);
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double bearing = atan2(y, x);
  bearing = degrees(bearing);
  return fmod((bearing + 360.0), 360.0);
}

double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2)
{
  const double R = 6371000.0;
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double deltaPhi = radians(lat2 - lat1);
  double deltaLambda = radians(lon2 - lon1);
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  return R * (2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
}

void drawArrow(int cx, int cy, int radius, double bearing)
{
  double angle = radians(bearing - 90);
  int tipX = cx + radius * cos(angle);
  int tipY = cy + radius * sin(angle);
  int baseX = cx - radius * 0.3 * cos(angle);
  int baseY = cy - radius * 0.3 * sin(angle);
  double wingAngle1 = angle + radians(150);
  double wingAngle2 = angle - radians(150);
  int wing1X = cx + radius * 0.6 * cos(wingAngle1);
  int wing1Y = cy + radius * 0.6 * sin(wingAngle1);
  int wing2X = cx + radius * 0.6 * cos(wingAngle2);
  int wing2Y = cy + radius * 0.6 * sin(wingAngle2);
  tft.fillTriangle(tipX, tipY, wing1X, wing1Y, baseX, baseY, TFT_GREEN);
  tft.fillTriangle(tipX, tipY, wing2X, wing2Y, baseX, baseY, TFT_GREEN);
  tft.fillCircle(cx, cy, 4, TFT_RED);
}

void handleConfigSubmission(std::map<String, String> &data)
{
  if (data["frequency"].length() > 0)
  {
    float freqMHz = data["frequency"].toFloat();
    long freqHz = (long)(freqMHz * 1000000.0);
    loraConfig.frequency = String(freqHz);
  }
  if (data["bandwidth"].length() > 0)
    loraConfig.bandwidth = data["bandwidth"];
  if (data["sync"].length() > 0)
    loraConfig.sync = data["sync"];
  long newBaud = data["baudrate"].toInt();
  if (newBaud > 0)
    loraConfig.baudrate = newBaud;
  if (data["cansatName"].length() > 0)
    loraConfig.cansatName = data["cansatName"];

  saveConfig();
  capportal.stop();
  delay(100);
  appState = STATE_INITIALIZING;
}

void enterConfigMode()
{
  String configHTML = generateConfigHTML(loraConfig);
  capportal.setHTML(configHTML);
  capportal.onSubmit(handleConfigSubmission);
  capportal.begin(SSID, PASSWRD, AP_TIMEOUT);
  apStartTime = millis();
  drawConfigScreen();
}

void loadConfig()
{
  loraConfig.frequency = preferences.getString("frequency", "868000000");
  loraConfig.bandwidth = preferences.getString("bandwidth", "125");
  loraConfig.sync = preferences.getString("sync", "12");
  loraConfig.baudrate = preferences.getLong("baudrate", 115200);
  loraConfig.cansatName = preferences.getString("cansatName", DEFAULT_CANSAT_NAME);
  cansatName = loraConfig.cansatName;
}

void saveConfig()
{
  preferences.putString("frequency", loraConfig.frequency);
  preferences.putString("bandwidth", loraConfig.bandwidth);
  preferences.putString("sync", loraConfig.sync);
  preferences.putLong("baudrate", loraConfig.baudrate);
  preferences.putString("cansatName", loraConfig.cansatName);
  cansatName = loraConfig.cansatName;
}

void initializeSDCard()
{
  if (!SD_ENABLED)
    return;
  spiSD.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  delay(100);
  if (!SD.begin(SD_CS, spiSD, 4000000))
  {
    sdCardAvailable = false;
    return;
  }
  if (SD.cardType() == CARD_NONE)
  {
    sdCardAvailable = false;
    SD.end();
    return;
  }
  sdCardAvailable = true;
  if (!SD.exists("/logs"))
    SD.mkdir("/logs");
  int fileCount = countSDFiles("/logs");
  currentLogFile = "/logs/telemetry_" + String(fileCount + 1) + ".csv";
  File file = SD.open(currentLogFile, FILE_WRITE);
  if (file)
  {
    file.println("Time,CS_Lat,CS_Lon,CS_Alt,CS_Speed,CS_Valid,GP_Lat,GP_Lon,GP_Alt,GP_Speed,GP_Sats,GP_Fix,Distance,Bearing");
    file.close();
  }
}

int countSDFiles(const char *dirname)
{
  File dir = SD.open(dirname);
  if (!dir || !dir.isDirectory())
    return 0;
  int count = 0;
  File file = dir.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
      count++;
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
  return count;
}

String buildTelemetryString()
{
  String data = String(millis()) + ",";
  data += String(canSatData.latitude, 6) + "," + String(canSatData.longitude, 6) + "," + String(canSatData.altitude, 2) + "," + String(canSatData.speed, 2) + "," + (canSatData.valid ? "1" : "0") + ",";
  data += String(gpsData.latitude, 6) + "," + String(gpsData.longitude, 6) + "," + String(gpsData.altitude, 2) + "," + String(gpsData.speed, 2) + "," + String(gpsData.satellites) + "," + (gpsData.hasFix ? "1" : "0") + ",";
  if (gpsData.hasFix && canSatData.valid)
  {
    double dist = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    double bear = calculateBearing(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    data += String(dist, 2) + "," + String(bear, 2);
  }
  else
    data += "0.00,0.00";
  return data;
}

void writeToSD()
{
  if (!sdCardAvailable || currentLogFile.length() == 0)
    return;
  String data = buildTelemetryString();
  File file = SD.open(currentLogFile, FILE_APPEND);
  if (file)
  {
    file.println(data);
    file.close();
  }
  else
    sdCardAvailable = false;
}

void drawSolarPanels(int cx, int cy, int radius)
{
  int panelLength = 50;
  int panelWidth = 15;
  double angle1 = radians(45 - 90);
  double angle2 = radians(225 - 90);
  int tr_startX = cx + (radius + 3) * cos(angle1);
  int tr_startY = cy + (radius + 3) * sin(angle1);
  int bl_startX = cx + (radius + 3) * cos(angle2);
  int bl_startY = cy + (radius + 3) * sin(angle2);
  for (int i = 0; i < panelLength; i++)
  {
    int x = tr_startX + i * cos(angle1);
    int y = tr_startY + i * sin(angle1);
    for (int j = -panelWidth / 2; j < panelWidth / 2; j++)
    {
      int px = x + j * cos(angle1 + radians(90));
      int py = y + j * sin(angle1 + radians(90));
      tft.drawPixel(px, py, 0x0014);
    }
  }
  for (int i = 0; i < panelLength; i++)
  {
    int x = bl_startX + i * cos(angle2);
    int y = bl_startY + i * sin(angle2);
    for (int j = -panelWidth / 2; j < panelWidth / 2; j++)
    {
      int px = x + j * cos(angle2 + radians(90));
      int py = y + j * sin(angle2 + radians(90));
      tft.drawPixel(px, py, 0x0014);
    }
  }
}

void readLoRaSerial()
{
  static String lineBuffer = "";
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c == '\n')
    {
      lineBuffer.trim();
      if (lineBuffer.length() > 0)
      {
        Serial.println(lineBuffer);
        parseLoRaData(lineBuffer);
      }
      lineBuffer = "";
    }
    else if (c != '\r')
    {
      lineBuffer += c;
      if (lineBuffer.length() > 400)
        lineBuffer = "";
    }
  }
}