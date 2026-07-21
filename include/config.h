#pragma once
#include <Arduino.h>

// Display size
constexpr int SCREEN_WIDTH = 480;
constexpr int SCREEN_HEIGHT = 320;

// Running & config screen
constexpr int SKIP_BTN_W = 110;
constexpr int SKIP_BTN_H = 36;
constexpr int SKIP_BTN_X = 382 - SKIP_BTN_W / 2;
constexpr int SKIP_BTN_Y = 275;
constexpr int VOL_ICON_X = 370, VOL_ICON_Y = 20, VOL_ICON_W = 36, VOL_ICON_H = 32;
constexpr int GEAR_ICON_X = 395;
constexpr int GEAR_ICON_W = 32, GEAR_ICON_H = 32;
constexpr int GEAR_ICON_Y = SCREEN_HEIGHT - GEAR_ICON_H;
constexpr int COMPASS_CX = 380, COMPASS_CY = 160, COMPASS_R = 70, ARROW_R = 60, COMPASS_CLEAR_R = 66;

// Display & touch
constexpr uint8_t TFT_CS = 2;
constexpr uint8_t TFT_RST = 3;
constexpr uint8_t TFT_DC = 4;
constexpr uint8_t TFT_MOSI = 9;
constexpr uint8_t TFT_MISO = 8;
constexpr uint8_t TFT_CLK = 7;
constexpr uint8_t TOUCH_CS = 5;
constexpr uint8_t TOUCH_IRQ = 6;

// SD
constexpr uint8_t SD_CS = 38;
constexpr uint8_t SD_MOSI = 11;
constexpr uint8_t SD_MISO = 12;
constexpr uint8_t SD_CLK = 13;
constexpr bool SD_ENABLED = true;

// LoRa UART
constexpr uint8_t LORA_UART_RX = 41;
constexpr uint8_t LORA_UART_TX = 42;

// Buzzer
constexpr uint8_t BUZZER = 1;

// GPS
constexpr uint8_t GPS_TX = 40;
constexpr uint8_t GPS_RX = 39;
constexpr double HDOP_GOOD = 2.0;
constexpr double HDOP_OK = 6.0;

// Timings
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200;
constexpr unsigned long TITLE_DURATION = 3000;
constexpr unsigned long LORA_TIMEOUT = 5000;
constexpr unsigned long ANIMATION_UPDATE_INTERVAL = 50;
constexpr unsigned long SD_WRITE_INTERVAL = 1000;

// Buzzer distance
constexpr double MAX_DISTANCE = 3000.0; // meter
constexpr double MIN_DISTANCE = 30.0;   // meter
constexpr unsigned long MAX_BEEP_INTERVAL = 2000;
constexpr unsigned long MIN_BEEP_INTERVAL = 50;

// Captive Portal AP
constexpr char SSID[] = "CanSat-Finder";
constexpr char PASSWRD[] = "seatfinder";
constexpr unsigned long AP_TIMEOUT = 180000; // 3 minutes

// Default cansat name
constexpr char DEFAULT_CANSAT_NAME[] = "SAS-MK3";

enum AppState
{
  STATE_TITLE,
  STATE_CONFIG,
  STATE_INITIALIZING,
  STATE_RUNNING
};

struct LoRaConfig
{
  String frequency;
  String bandwidth;
  String sync;
  long baudrate;
};

struct GPSData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  int satellites = 0;
  double hdop = 0.0;
  bool hasFix = false;
  double course = 0.0;
};

struct CanSatData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  bool valid = false;
  unsigned long lastReceived = 0;
  String lastReceivedTimeStr = "--:--:--";
};

struct PreviousValues
{
  double csLat = -999, csLon = -999, csAlt = -999, csSpeed = -999;
  String csLastTime = "";
  double gpLat = -999, gpLon = -999, gpAlt = -999, gpSpeed = -999;
  bool csValid = false;
  double distance = -1, bearing = -999, course = -999;
};