#pragma once

#include <Arduino.h>

// Display
#define DISPLAY_UPDATE_INTERVAL 200

// GPS
#define GPS_RX 5
#define GPS_TX 6

// LoRa
#define LORA_RX 18
#define LORA_TX 17

// Buzzer
#define BUZZER 7

// SD Card
#define SD_ENABLED true
#define SD_CS_PIN 3
#define SD_SCK_PIN 14
#define SD_MISO_PIN 15
#define SD_MOSI_PIN 13

// Captive portal
const char SSID[] = "CanSat-Finder";
const char PASSWRD[] = "seatfinder";
#define AP_TIMEOUT 300000 // 5 minutes

// LoRa configuration structure
struct LoRaConfig
{
  String frequency;
  String bandwidth;
  String sync;
  long baudrate;
};