#pragma once

// Display
#define DISPLAY_UPDATE_INTERVAL 200
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// GPS
#define GPS_RX 5
#define GPS_TX 6
#define MIN_SPEED_FOR_HEADING 1.0 // km/h

// LoRa
#define LORA_RX 18
#define LORA_TX 17

// Buzzer
#define BUZZER 7

// SD Card (Optional - use -1 to disable)
#define SD_ENABLED true // Set to false if no SD card
#define SD_CS_PIN 3
#define SD_SCK_PIN 14   // Add these for custom SPI pins
#define SD_MISO_PIN 15  // Based on your working example
#define SD_MOSI_PIN 13  // Based on your working example

// Configuration mode
#define CONFIG_BUTTON_PIN 4
#define CONFIG_BUTTON_HOLD_TIME 3000
const char SSID[] = "CanSat-Finder";
const char PASSWRD[] = "seatfinder";
#define AP_TIMEOUT 300000 // 5 minutes

// Default LoRa settings
struct LoRaConfig
{
  String frequency = "865375000";
  String bandwidth = "250";
  String sync = "12";
  long baudrate = 115200;
};

#endif // CONFIG_H