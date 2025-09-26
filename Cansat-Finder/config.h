#pragma once

#ifndef CONFIG_H
#define CONFIG_H

// Display
constexpr byte TFT_CS = 5;
constexpr byte TFT_DC = 2;
constexpr byte TFT_RST = 4;
constexpr byte TFT_SCLK = 18;
constexpr byte TFT_MOSI = 23;
constexpr unsigned long DISPLAY_UPDATE_INTERVAL = 200;

// GPS
constexpr byte GPS_RX = 13;
constexpr float MIN_SPEED_FOR_HEADING = 2.0;  // km/h

// Configuration mode
constexpr byte CONFIG_BUTTON_PIN = 0;  // Boot button
constexpr unsigned long CONFIG_BUTTON_HOLD_TIME = 3000;
constexpr char SSID[] = "SAS-MK3-FINDER";
constexpr char PASSWRD[] = "seatsasmk3";
constexpr unsigned long AP_TIMEOUT = 300000; // 5 minutes

// QMC5883L registers
constexpr byte QMC5883L_ADDR = 0x0D;
constexpr byte QMC5883L_X_LSB = 0x00;
constexpr byte QMC5883L_STATUS = 0x06;
constexpr byte QMC5883L_CONFIG = 0x09;
constexpr byte QMC5883L_CONFIG2 = 0x0A;
constexpr byte QMC5883L_RESET = 0x0B;

// Default LoRa settings
struct LoRaConfig {
  String frequency = "865375000";
  String bandwidth = "250";
  String sync = "12";
  long baudrate = 115200;
};

#endif  // CONFIG_H