#pragma once
#include <Arduino.h>

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

// Timings
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200;
constexpr unsigned long TITLE_DURATION = 3000;
constexpr unsigned long LORA_TIMEOUT = 5000;
constexpr unsigned long ANIMATION_UPDATE_INTERVAL = 50; 
constexpr unsigned long SD_WRITE_INTERVAL = 1000;

// Buzzer distance 
constexpr double MAX_DISTANCE = 3000.0; // meter    
constexpr double MIN_DISTANCE = 30.0; // meter       
constexpr unsigned long MAX_BEEP_INTERVAL = 10000; 
constexpr unsigned long MIN_BEEP_INTERVAL = 500; 


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
  String cansatName;
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
  double csLat = -999;
  double csLon = -999;
  double csAlt = -999;
  double csSpeed = -999;
  String csLastTime = "";
  double gpLat = -999;
  double gpLon = -999;
  double gpAlt = -999;
  double gpSpeed = -999;
  int satellites = -1;
  bool gpsFix = false;
  bool csValid = false;
  double distance = -1;
  double bearing = -999;
  double course = -999;
};