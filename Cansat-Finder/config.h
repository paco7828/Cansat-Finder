#pragma once

// Display
constexpr TFT_CS = 5;
constexpr TFT_DC = 2;
constexpr TFT_RST = 4;
constexpr TFT_SCLK = 18;
constexpr TFT_MOSI = 23;
constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 200;

// GPS
constexpr GPS_RX = 13;
constexpr float MIN_SPEED_FOR_HEADING = 2.0;  // km/h minimum speed for reliable GPS heading

// Configuration mode
constexpr CONFIG_BUTTON_PIN = 0; // Boot button for entering configuration mode
constexpr unsigned long CONFIG_BUTTON_HOLD_TIME = 3000;

// QMC5883L registers
constexpr QMC5883L_ADDR = 0x0D;
constexpr QMC5883L_X_LSB = 0x00;
constexpr QMC5883L_STATUS = 0x06;
constexpr QMC5883L_CONFIG = 0x09;
constexpr QMC5883L_CONFIG2 = 0x0A;
constexpr QMC5883L_RESET = 0x0B;

// Default LoRa settings
struct LoRaConfig {
  String frequency = "865375000";
  String bandwidth = "250";
  String sync = "12";
  long baudrate = 115200;
};