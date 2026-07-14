// src/config.h
#pragma once
#include <Arduino.h>

// LovyanGFX Kijelzo es Touch pitek (touch-display-test.ino alapjan)
constexpr uint8_t TFT_CS = 2;
constexpr uint8_t TFT_RST = 3;
constexpr uint8_t TFT_DC = 4;
constexpr uint8_t TFT_MOSI = 9;
constexpr uint8_t TFT_MISO = 8;
constexpr uint8_t TFT_CLK = 7;
constexpr uint8_t TOUCH_CS = 5;
constexpr uint8_t TOUCH_IRQ = 6;

// SD Kartya pinout (A LovyanGFX-es megosztott busz miatt atpakolva a p_csaba)
constexpr uint8_t SD_CS = 38;
constexpr uint8_t SD_MOSI = 11;
constexpr uint8_t SD_MISO = 12;
constexpr uint8_t SD_CLK = 13;

// LoRa UART a HT-CT62 masodlagos proci felol
constexpr uint8_t LORA_UART_RX = 41;  // <- HT-CT62 TX (GPIO21)
constexpr uint8_t LORA_UART_TX = 42;  // -> HT-CT62 RX (GPIO20)

// FIGYELEM: A BUZZER a 7-es pinen volt, de az most a TFT_CLK! Atraktam az 1-esre!
constexpr uint8_t BUZZER = 1;

// FIGYELEM: A GPS az 5, 6-os pinen volt, de azokat behuzta a TOUCH! Atraktam a 43, 44-esre!
constexpr uint8_t GPS_TX = 40;
constexpr uint8_t GPS_RX = 39;

// Időzítések
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200; // ms
constexpr unsigned long TITLE_DURATION = 3000;    // 3 mp intro
constexpr unsigned long LORA_TIMEOUT = 5000;      // 5 mp timeout

// Animacio config modban
constexpr unsigned long ANIMATION_UPDATE_INTERVAL = 50; 

// Buzzer tavolsag kuszobok
constexpr double MAX_DISTANCE = 3000.0;            
constexpr double MIN_DISTANCE = 30.0;              
constexpr unsigned long MAX_BEEP_INTERVAL = 10000; 
constexpr unsigned long MIN_BEEP_INTERVAL = 500;   

// SD Beallitasok
constexpr bool SD_ENABLED = true;
constexpr unsigned long SD_WRITE_INTERVAL = 1000; 

// Captive Portal AP adatok
constexpr char SSID[] = "CanSat-Finder";
constexpr char PASSWRD[] = "seatfinder";
constexpr unsigned long AP_TIMEOUT = 180000; // 3 perc

// Alapertelmezett nev
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