#pragma once

#include <Arduino.h>

// Display
constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200; // ms
unsigned long lastDisplayUpdate = 0;
unsigned long titleStartTime = 0;
constexpr unsigned long TITLE_DURATION = 3000; // 3 seconds
constexpr unsigned long LORA_TIMEOUT = 5000;   // 5 seconds

// Animation at Config mode
float animationAngle = 0.0;
unsigned long lastAnimationUpdate = 0;
constexpr unsigned long ANIMATION_UPDATE_INTERVAL = 50; // 50ms = 20 FPS

// GPS
constexpr uint8_t GPS_RX = 5;
constexpr uint8_t GPS_TX = 6;

// LoRa
constexpr uint8_t LORA_RX = 18;
constexpr uint8_t LORA_TX = 17;

// Buzzer
constexpr uint8_t BUZZER = 7;
unsigned long lastBeepTime = 0;
unsigned long beepInterval = 0;
bool beepActive = false;
unsigned long beepStartTime = 0;
int beepDuration = 0;
int beepFrequency = 0;

// Distance thresholds for beeping
constexpr double MAX_DISTANCE = 3000.0;            // 5000 meters
constexpr double MIN_DISTANCE = 30.0;              // 30 meters
constexpr unsigned long MAX_BEEP_INTERVAL = 10000; // 10 seconds
constexpr unsigned long MIN_BEEP_INTERVAL = 500;   // 0.5 seconds

// SD Card
constexpr bool SD_ENABLED = true;
constexpr uint8_t SD_CS_PIN = 3;
constexpr uint8_t SD_SCK_PIN = 14;
constexpr uint8_t SD_MISO_PIN = 15;
constexpr uint8_t SD_MOSI_PIN = 13;
bool sdCardAvailable = false;
unsigned long lastSDWrite = 0;
constexpr unsigned long SD_WRITE_INTERVAL = 1000; // 1 second
String currentLogFile = "";

// Captive portal
constexpr char SSID[] = "CanSat-Finder";
constexpr char PASSWRD[] = "seatfinder";
constexpr unsigned long AP_TIMEOUT = 300000; // 5 minutes

// Device name
constexpr char DEFAULT_CANSAT_NAME[] = "SAS-MK3";
String cansatName = DEFAULT_CANSAT_NAME;

// Initialization
unsigned long initStartTime = 0;
int initStep = 0;
constexpr unsigned long INIT_STEP_DURATION = 1000; // 1 second

// Config mode
int lastClientCount = -1;

// System state
enum AppState
{
  STATE_TITLE,
  STATE_CONFIG,
  STATE_INITIALIZING,
  STATE_RUNNING
};
AppState appState = STATE_TITLE;

// LoRa configuration structure
struct LoRaConfig
{
  String frequency;
  String bandwidth;
  String sync;
  long baudrate;
  String cansatName;
};

// Local GPS data
struct GPSData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0; // m/s
  int satellites = 0;
  double hdop = 0.0;
  bool hasFix = false;
  double course = 0.0;
} gpsData;

// CanSat data from LoRa
struct CanSatData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  bool valid = false;
  unsigned long lastReceived = 0;
} canSatData;

// Previous values for selective update
struct PreviousValues
{
  double csLat = -999;
  double csLon = -999;
  double csAlt = -999;
  double csSpeed = -999;
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
} prevVals;