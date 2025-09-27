// Libs & Headers
#include "config.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "Better-GPS.h"
#include "LoRaRx.h"
#include "Better-Capportal.h"
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <captive-portal-web.h>
#include <MPU6050.h>

// Instances
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
BetterGPS gps;
LoRaRx *loraRx = nullptr;  // Will be created with dynamic settings
LoRaConfig loraConfig;
BetterCapportal capportal;
MPU6050 mpu;
Preferences preferences;

// Track configuration mode
bool configMode = false;

// Calculation variables
double heading = 0;
double prevAngle = 0;
double angle = 0;
double latYou = 0, lonYou = 0;
double latCS = 0, lonCS = 0;

// Track valid received coordinates
bool validCSPos = false;

// Track magnetometer working status
bool magnetometerWorking = false;

// Calibration containers
int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
int16_t zMin = 32767, zMax = -32768;

// Offsets & scales
int16_t xOffset = 0, yOffset = 0, zOffset = 0;
double xScale = 1.0f, yScale = 1.0f, zScale = 1.0f;

// Accelerometer
double deviceTiltX = 0, deviceTiltY = 0;

// Helper variables for magnetometer
double filteredHeading = -1.0f;
bool calibrationDone = false;

// GPS heading fallback variables
double gpsHeading = 0;
bool hasGpsHeading = false;
unsigned long lastGpsUpdate = 0;

// Display variables
int meterValue;
int centerX;
int centerY = 55;  // arrow center
unsigned long lastDisplayUpdate = 0;

// Function prototypes
void saveLoRaConfig();
void loadLoRaConfig();
void handleConfigSubmission(std::map<String, String> &data);
void enterConfigMode();
void initializeLoRa();
void checkConfigButton();
void initializeDisplay();
void drawStatusIndicator(bool hasGpsFix, bool hasCanSatData);
void updateDisplay();

/*
   _____ ______ _______ _    _ _____  
  / ____|  ____|__   __| |  | |  __ \ 
 | (___ | |__     | |  | |  | | |__) |
  \___ \|  __|    | |  | |  | |  ___/ 
  ____) | |____   | |  | |__| | |     
 |_____/|______|  |_|   \____/|_|       

*/
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== CanSat Finder Starting ===");

  // Initialize preferences
  preferences.begin("cansat", false);

  // Initialize config button
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);

  // Load LoRa configuration from flash
  loadLoRaConfig();

  // Initialize display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  centerX = tft.width() / 2;

  // Show startup message
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(25, 10);
  tft.println("SAS-MK3");
  tft.setCursor(30, 30);
  tft.println("Finder");
  tft.setCursor(5, 65);
  tft.println("Loading...");
  tft.setCursor(5, 110);
  tft.println("Hold BOOT for config");

  // Initialize GPS
  Serial.println("Initializing GPS...");
  gps.begin(GPS_RX);

  // Initialize LoRa with saved config
  initializeLoRa();

  // Initialize I2C & magnetometer
  Wire.begin(21, 22);
  initMagnetometer();

  // Clear startup message and draw fixed display elements
  delay(2000);
  initializeDisplay();
  Serial.println("Display initialized. System ready.");
}


/*
  _      ____   ____  _____  
 | |    / __ \ / __ \|  __ \ 
 | |   | |  | | |  | | |__) |
 | |   | |  | | |  | |  ___/ 
 | |___| |__| | |__| | |     
 |______\____/ \____/|_|     
                             
*/
void loop() {
  unsigned long currentTime = millis();

  // Check for config button press
  if (!configMode) {
    checkConfigButton();
  }

  // Handle captive portal if in config mode
  if (configMode) {
    capportal.handle();

    // Show client count on display
    static unsigned long lastClientUpdate = 0;
    if (currentTime - lastClientUpdate > 1000) {
      tft.fillRect(5, 150, tft.width() - 10, 10, ST77XX_BLACK);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(5, 152);
      tft.printf("Clients: %d", capportal.clientCount());
      lastClientUpdate = currentTime;
    }

    delay(50);
    return;  // Don't run normal operations in config mode
  }

  // Update GPS data
  gps.update();

  // Listen for LoRa data
  if (loraRx != nullptr) {
    String receivedData = loraRx->listen();

    if (receivedData.length() > 0) {
      Serial.println("Processing received data: " + receivedData);

      int semicolonIndex = receivedData.indexOf(';');
      if (semicolonIndex != -1) {
        String latStr = receivedData.substring(0, semicolonIndex);
        String lonStr = receivedData.substring(semicolonIndex + 1);

        double newLatCS = latStr.toDouble();
        double newLonCS = lonStr.toDouble();

        // Validate the coordinates (basic sanity check)
        if (newLatCS >= -90 && newLatCS <= 90 && newLonCS >= -180 && newLonCS <= 180 && (newLatCS != 0.0 || newLonCS != 0.0)) {
          latCS = newLatCS;
          lonCS = newLonCS;
          validCSPos = true;

          Serial.print("Valid CS coords received: ");
          Serial.print(latCS, 6);
          Serial.print(", ");
          Serial.println(lonCS, 6);
        } else {
          Serial.println("Invalid coordinates received, ignoring...");
        }
      } else {
        Serial.println("Invalid data format received: " + receivedData);
      }
    }
  }

  // Update display at regular intervals
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }

  // Print status every 5 seconds
  static unsigned long lastStatusPrint = 0;
  if (currentTime - lastStatusPrint >= 5000) {
    if (gps.hasFix()) {
      Serial.printf("GPS Fix: %.6f, %.6f, Speed: %.1f km/h\n", latYou, lonYou, gps.getSpeedKmph());
    } else {
      Serial.println("GPS: No fix");
    }

    // Print CanSat position status
    if (validCSPos) {
      Serial.printf("CanSat Position: %.6f, %.6f (Distance: %dm)\n", latCS, lonCS, meterValue);
    } else {
      Serial.println("CanSat Position: No data received");
    }

    // Print heading info
    if (magnetometerWorking) {
      int16_t mx, my, mz;
      readMagnetometer(mx, my, mz);
      Serial.printf("Magnetometer: X=%d, Y=%d, Z=%d, Heading=%.1f°\n", mx, my, mz, heading);
    } else {
      Serial.printf("Heading source: %s (%.1f°)\n", hasGpsHeading ? "GPS" : "North", heading);
    }

    // Print LoRa config status
    Serial.printf("LoRa Config: Freq=%s, BW=%s, Sync=%s, Baud=%ld\n",
                  loraConfig.frequency.c_str(), loraConfig.bandwidth.c_str(),
                  loraConfig.sync.c_str(), loraConfig.baudrate);

    lastStatusPrint = currentTime;
  }

  delay(50);
}

/*
  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
 |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
 | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___
 |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
 | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) | 
 |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/  
                                                                                                            
*/

//  ---------------------------------------------------------------
// ----------------- CONFIGURATION MODE ---------------------------
//  ---------------------------------------------------------------

// Handle configuration mode form submission
void handleConfigSubmission(std::map<String, String> &data) {
  Serial.println("LoRa configuration received:");

  // Extract and validate data
  String newFreq = data["frequency"];
  String newBW = data["bandwidth"];
  String newSync = data["sync"];
  String newBaudStr = data["baudrate"];

  // Validate frequency (should be a number)
  if (newFreq.length() > 0 && newFreq.toInt() > 0) {
    loraConfig.frequency = newFreq;
  }

  // Validate bandwidth
  if (newBW.length() > 0) {
    loraConfig.bandwidth = newBW;
  }

  // Validate sync word (hex format)
  if (newSync.length() > 0) {
    loraConfig.sync = newSync;
  }

  // Validate baudrate
  long newBaud = newBaudStr.toInt();
  if (newBaud > 0) {
    loraConfig.baudrate = newBaud;
  }

  // Print received configuration
  Serial.printf("New config - Freq: %s, BW: %s, Sync: %s, Baud: %ld\n",
                loraConfig.frequency.c_str(), loraConfig.bandwidth.c_str(),
                loraConfig.sync.c_str(), loraConfig.baudrate);

  // Save to flash
  saveLoRaConfig();

  // Reinitialize LoRa with new settings
  initializeLoRa();

  // Exit config mode & stop access point hosting
  configMode = false;
  capportal.stop();

  // Show success message on display
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.println("Config Updated!");
  tft.setCursor(10, 25);
  tft.println("LoRa reinitialized");
  tft.setCursor(10, 40);
  tft.println("Back to normal");

  delay(3000);
  initializeDisplay();
}

// Enter configuration mode to set LoRa params
void enterConfigMode() {
  // Check and set configMode
  if (configMode) return;
  configMode = true;

  // Show config mode on display
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(5, 10);
  tft.println("CONFIG MODE");
  tft.setCursor(5, 25);
  tft.println("Connect to WiFi:");
  tft.setCursor(5, 40);
  tft.println("SAS-MK3-Finder");
  tft.setCursor(5, 55);
  tft.println("Go to: 4.3.2.1");
  tft.setCursor(5, 75);
  tft.println("Current settings:");
  tft.setCursor(5, 90);
  tft.printf("Freq: %s", loraConfig.frequency.c_str());
  tft.setCursor(5, 105);
  tft.printf("BW: %s", loraConfig.bandwidth.c_str());
  tft.setCursor(5, 120);
  tft.printf("Sync: %s", loraConfig.sync.c_str());
  tft.setCursor(5, 135);
  tft.printf("Baud: %ld", loraConfig.baudrate);

  // Generate HTML with current config values
  String configHTML = generateConfigHTML(loraConfig);

  // Set up captive portal
  capportal.setHTML(configHTML);
  capportal.onSubmit(handleConfigSubmission);

  // Start AP without password for easy access
  capportal.begin(SSID, PASSWRD, AP_TIMEOUT);

  Serial.println("Configuration portal started!");
  Serial.println("Connect to WiFi: SAS-MK3-finder");
  Serial.println("Then go to: http://4.3.2.1");
}

// Handle config button state with debug output
void checkConfigButton() {
  // Default variables
  static bool buttonPressed = false;
  static unsigned long pressStartTime = 0;
  static unsigned long lastDebugPrint = 0;

  bool currentState = !digitalRead(CONFIG_BUTTON_PIN);  // Active low

  // Debug: Print button state every 2 seconds
  if (millis() - lastDebugPrint > 2000) {
    int rawReading = digitalRead(CONFIG_BUTTON_PIN);
    Serial.printf("Button debug - Raw reading: %d, Current state: %d, Button pressed: %d\n",
                  rawReading, currentState, buttonPressed);
    lastDebugPrint = millis();
  }

  if (currentState && !buttonPressed) {
    // Button just pressed
    buttonPressed = true;
    pressStartTime = millis();
    Serial.println("Button PRESSED!");
  } else if (!currentState && buttonPressed) {
    // Button released
    buttonPressed = false;
    unsigned long pressDuration = millis() - pressStartTime;
    Serial.printf("Button RELEASED after %lu ms\n", pressDuration);

    if (pressDuration >= CONFIG_BUTTON_HOLD_TIME) {
      // Long press detected => enter config mode if haven't already
      if (!configMode) {
        Serial.println("Long press detected - entering config mode!");
        enterConfigMode();
      }
    } else {
      Serial.println("Short press - ignoring");
    }
  }

  // Show progress on display while button is held
  if (buttonPressed) {
    unsigned long currentDuration = millis() - pressStartTime;
    Serial.printf("Button held for %lu ms\n", currentDuration);

    if (currentDuration >= CONFIG_BUTTON_HOLD_TIME && !configMode) {
      // Will enter config mode on release
      Serial.println("Ready to enter config mode on release");
    } else if (currentDuration > 1000) {
      // Show progress
      static unsigned long lastProgressUpdate = 0;
      if (millis() - lastProgressUpdate > 100) {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.fillRect(0, 25, tft.width(), 10, ST77XX_BLACK);
        tft.setCursor(10, 27);
        tft.printf("Hold for config: %d%%", (int)((currentDuration * 100) / CONFIG_BUTTON_HOLD_TIME));
        lastProgressUpdate = millis();
      }
    }
  }
}

//  ---------------------------------------------------------------
// -------------------- INITIALIZATION ----------------------------
//  ---------------------------------------------------------------

// Initialize display with template
void initializeDisplay() {
  tft.fillScreen(ST77XX_BLACK);

  // Only draw the coordinate labels that never change
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Finder's coordinates labels
  tft.setCursor(0, 90);
  tft.print("Lat (you): ");
  tft.setCursor(0, 105);
  tft.print("Lon (you): ");

  // Separator line
  tft.drawLine(0, 120, tft.width(), 120, ST77XX_WHITE);

  // CanSat coordinates labels
  tft.setCursor(0, 130);
  tft.print("Lat (CS): ");
  tft.setCursor(0, 145);
  tft.print("Lon (CS): ");
}

// Initialize LoRa with current configuration
void initializeLoRa() {
  // Delete existing instance if it exists
  if (loraRx != nullptr) {
    delete loraRx;
  }

  // Create new LoRa instance
  loraRx = new LoRaRx(16, 17, loraConfig.baudrate);
  loraRx->begin(loraConfig.frequency, loraConfig.bandwidth, loraConfig.sync);
  Serial.println("LoRa initialized successfully!");
}

// Intiailize magnetometer from GY-87 through MPU6050
void initMagnetometer() {
  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");

    // Enable I2C bypass to access magnetometer
    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);
    delay(100);

    // Reset QMC5883L
    writeRegister(QMC5883L_ADDR, QMC5883L_RESET, 0x01);
    delay(100);

    // Configure QMC5883L - Mode: Continuous, ODR: 200Hz, RNG: 2G, OSR: 64
    writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG, 0x1D);
    delay(10);

    // Set interrupt enable
    writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG2, 0x00);
    delay(10);

    // Verify configuration
    uint8_t config = readRegister(QMC5883L_ADDR, QMC5883L_CONFIG);
    Serial.print("QMC5883L Config register: 0x");
    Serial.println(config, HEX);

    if (config == 0x1D) {
      magnetometerWorking = true;
      Serial.println("QMC5883L initialized successfully!");

      // Start calibration
      performMagnetometerCalibration();
    } else {
      Serial.println("QMC5883L configuration failed");
      magnetometerWorking = false;
    }
  } else {
    Serial.println("MPU6050 connection failed");
    magnetometerWorking = false;
  }
}

//  ---------------------------------------------------------------
// --------------- Read / Load & Write / Save ---------------------
//  ---------------------------------------------------------------

// Load LoRa configuration from flash (preferences)
void loadLoRaConfig() {
  loraConfig.frequency = preferences.getString("frequency", "865375000");
  loraConfig.bandwidth = preferences.getString("bandwidth", "250");
  loraConfig.sync = preferences.getString("sync", "12");
  loraConfig.baudrate = preferences.getLong("baudrate", 115200);
  Serial.printf("Loaded config - Freq: %s, BW: %s, Sync: %s, Baud: %ld\n",
                loraConfig.frequency.c_str(), loraConfig.bandwidth.c_str(),
                loraConfig.sync.c_str(), loraConfig.baudrate);
}

// Save LoRa configuration to flash (preferences)
void saveLoRaConfig() {
  preferences.putString("frequency", loraConfig.frequency);
  preferences.putString("bandwidth", loraConfig.bandwidth);
  preferences.putString("sync", loraConfig.sync);
  preferences.putLong("baudrate", loraConfig.baudrate);
  Serial.println("Configuration saved successfully!");
}

// Write register value
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read register value
uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool readMagnetometerRaw(int16_t *x, int16_t *y, int16_t *z) {
  if (!magnetometerWorking) return false;

  // Check if data is ready
  uint8_t status = readRegister(QMC5883L_ADDR, QMC5883L_STATUS);
  if (status == 0xFF || !(status & 0x01)) {
    return false;
  }

  // Read 6 bytes
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_X_LSB);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom(QMC5883L_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint8_t xLow = Wire.read();
    uint8_t xHigh = Wire.read();
    uint8_t yLow = Wire.read();
    uint8_t yHigh = Wire.read();
    uint8_t zLow = Wire.read();
    uint8_t zHigh = Wire.read();

    *x = (int16_t)(xHigh << 8 | xLow);
    *y = (int16_t)(yHigh << 8 | yLow);
    *z = (int16_t)(zHigh << 8 | zLow);
    return true;
  }
  return false;
}

// Read calibrated magnetometer values (for heading calculation)
void readMagnetometer(int16_t &x, int16_t &y, int16_t &z) {
  if (!magnetometerWorking || !calibrationDone) {
    x = y = z = 0;
    return;
  }

  // Average multiple samples
  long sx = 0, sy = 0, sz = 0;
  int successfulReads = 0;

  for (int i = 0; i < N_SAMPLES; ++i) {
    int16_t rawX, rawY, rawZ;
    if (readMagnetometerRaw(&rawX, &rawY, &rawZ)) {
      sx += (long)rawX;
      sy += (long)rawY;
      sz += (long)rawZ;
      successfulReads++;
    }
    delay(10);
  }

  if (successfulReads > 0) {
    double xAvg = (double)sx / (double)successfulReads;
    double yAvg = (double)sy / (double)successfulReads;
    double zAvg = (double)sz / (double)successfulReads;

    // Apply calibration
    x = (int16_t)((xAvg - (double)xOffset) * xScale);
    y = (int16_t)((yAvg - (double)yOffset) * yScale);
    z = (int16_t)((zAvg - (double)zOffset) * zScale);
  } else {
    x = y = z = 0;
  }
}

//  ---------------------------------------------------------------
// ------------------------ DRAWINGS ------------------------------
//  ---------------------------------------------------------------

// Unified status indicator
void drawStatusIndicator(bool hasGpsFix, bool hasCanSatData) {
  // Clear the center area
  tft.fillRect(centerX - 30, centerY - 16, 60, 32, ST77XX_BLACK);

  if (!hasGpsFix) {
    // No GPS - show question mark
    tft.setTextSize(4);
    tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
    int charWidth = 6 * 4;
    int charHeight = 8 * 4;
    tft.setCursor(centerX - charWidth / 2, centerY - charHeight / 2);
    tft.print("?");
  } else if (!hasCanSatData) {
    // GPS but no CanSat data
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(centerX - 20, centerY - 8);
    tft.print("No CS");
  }
}

// Improved meter drawing with consistent positioning
void drawMeter() {
  // Always clear the same meter area
  tft.fillRect(0, 0, tft.width(), 25, ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Determine display string
  String displayStr;
  if (meterValue == 0) {
    displayStr = "-- m";
  } else if (meterValue < 1000) {
    displayStr = String(meterValue) + " m";
  } else {
    double km = meterValue / 1000.0;
    displayStr = String(km, 2) + " km";
  }

  // Center the text
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(displayStr, 0, 0, &x1, &y1, &w, &h);
  int16_t xPos = (tft.width() - w) / 2;

  // Actual value
  tft.setCursor(xPos, 5);
  tft.print(displayStr);
}

// Update display with new data
void updateDisplay() {
  bool hasGpsFix = gps.hasFix();

  if (hasGpsFix) {
    latYou = gps.getLatitude();
    lonYou = gps.getLongitude();

    // Determine heading source with GPS PRIORITY (fixes rotation issue)
    bool usingMagnetometer = false;
    double currentSpeed = gps.getSpeedKmph();

    // PRIORITY 1: GPS heading when moving (most reliable for navigation)
    if (currentSpeed > MIN_SPEED_FOR_HEADING && gps.isCourseValid()) {
      gpsHeading = gps.getCourseDeg();
      hasGpsHeading = true;
      lastGpsUpdate = millis();
      heading = gpsHeading;
      Serial.printf("Using GPS heading: %.1f° (Speed: %.1f km/h)\n", heading, currentSpeed);
    }
    // PRIORITY 2: Recent GPS heading (within last 15 seconds)
    else if (hasGpsHeading && (millis() - lastGpsUpdate < 15000)) {
      heading = gpsHeading;
      Serial.printf("Using recent GPS heading: %.1f° (age: %lu ms)\n",
                    heading, millis() - lastGpsUpdate);
    }
    // PRIORITY 3: Magnetometer (with warning about device orientation)
    else if (magnetometerWorking && calibrationDone) {
      int16_t mx, my, mz;
      readMagnetometer(mx, my, mz);

      if (mx != 0 || my != 0) {
        double currentHeading = calculateMagneticHeading(mx, my);
        heading = currentHeading;
        usingMagnetometer = true;

        // Add warning about device orientation dependency
        Serial.printf("Using magnetometer heading: %.1f° (WARNING: affected by device rotation)\n", heading);
      }
    }
    // PRIORITY 4: Last resort - assume North
    else {
      heading = 0;
      hasGpsHeading = false;
      Serial.println("Using North assumption (0°) - move device to get GPS heading");
    }

    if (validCSPos) {
      double bearingToCS = calculateBearing(latYou, lonYou, latCS, lonCS);

      // FIXED: Calculate relative angle correctly
      double relativeAngle = bearingToCS - heading;

      // Normalize to 0-360 range
      while (relativeAngle < 0) relativeAngle += 360;
      while (relativeAngle >= 360) relativeAngle -= 360;

      meterValue = calculateDistance(latYou, lonYou, latCS, lonCS);

      Serial.printf("Bearing to CS: %.1f°, Device heading: %.1f°, Arrow angle: %.1f°\n",
                    bearingToCS, heading, relativeAngle);

      // Clear previous arrow
      tft.fillCircle(centerX, centerY, 35, ST77XX_BLACK);

      // Determine arrow color based on heading source reliability
      uint16_t arrowColor;
      if (!usingMagnetometer && hasGpsHeading) {
        arrowColor = ST77XX_GREEN;  // Green for GPS heading (most reliable)
      } else if (usingMagnetometer) {
        arrowColor = ST77XX_YELLOW;  // Yellow for magnetometer (device-dependent)
      } else {
        arrowColor = ST77XX_RED;  // Red for North assumption (unreliable)
      }

      // Draw arrow
      drawRotatingArrow(centerX, centerY, 22, 30, relativeAngle, arrowColor);
      prevAngle = relativeAngle;
    } else {
      // GPS fix but no CanSat data - clear arrow and show status
      tft.fillCircle(centerX, centerY, 35, ST77XX_BLACK);
      drawStatusIndicator(true, false);
      meterValue = 0;
    }

    // Update coordinates display when GPS is available
    drawCoordinates();
  } else {
    // No GPS fix - clear arrow and show status
    tft.fillCircle(centerX, centerY, 35, ST77XX_BLACK);
    drawStatusIndicator(false, false);
    meterValue = 0;
  }

  // Always update these regardless of GPS status
  drawMeter();
  drawCanSatCoordinates();
}

// Arrow pointing to received coords' direction
void drawRotatingArrow(int cx, int cy, int headSize, int stickLen, double angleDeg, uint16_t color) {
  // Convert angle to radians (0° = North, clockwise positive)
  double rad = radians(angleDeg - 90);  // Subtract 90° so 0° points North (up)

  // Arrow head coordinates (pointing up initially)
  int x0 = 0, y0 = -headSize;                  // Tip of arrow
  int x1 = -headSize / 3, y1 = -headSize / 3;  // Left wing
  int x2 = headSize / 3, y2 = -headSize / 3;   // Right wing

  // Arrow stick coordinates
  int stickWidth = 4;
  int sx1 = -stickWidth / 2, sy1 = -headSize / 3;
  int sx2 = stickWidth / 2, sy2 = -headSize / 3;
  int sx3 = -stickWidth / 2, sy3 = stickLen / 2;
  int sx4 = stickWidth / 2, sy4 = stickLen / 2;

  // Rotation helper functions
  auto rotX = [&](int x, int y) -> int {
    return (int)(x * cos(rad) - y * sin(rad)) + cx;
  };
  auto rotY = [&](int x, int y) -> int {
    return (int)(x * sin(rad) + y * cos(rad)) + cy;
  };

  // Draw arrow head (triangle)
  tft.fillTriangle(
    rotX(x0, y0), rotY(x0, y0),  // tip
    rotX(x1, y1), rotY(x1, y1),  // left wing
    rotX(x2, y2), rotY(x2, y2),  // right wing
    color);

  // Draw arrow stick (rectangle)
  tft.fillTriangle(
    rotX(sx1, sy1), rotY(sx1, sy1),
    rotX(sx2, sy2), rotY(sx2, sy2),
    rotX(sx3, sy3), rotY(sx3, sy3),
    color);
  tft.fillTriangle(
    rotX(sx2, sy2), rotY(sx2, sy2),
    rotX(sx3, sy3), rotY(sx3, sy3),
    rotX(sx4, sy4), rotY(sx4, sy4),
    color);

  // Draw a small center dot for reference
  tft.fillCircle(cx, cy, 2, color);
}

// Finder's coordinates on display
void drawCoordinates() {
  // Clear previous coords
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.fillRect(65, 90, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 105, tft.width() - 65, 8, ST77XX_BLACK);

  // Real values from GPS
  if (gps.hasFix()) {
    tft.setCursor(65, 90);
    tft.print(latYou, 6);
    tft.setCursor(65, 105);
    tft.print(lonYou, 6);
  }
}

// Received coordinates on display
void drawCanSatCoordinates() {
  // Clear previous coords
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.fillRect(65, 130, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 145, tft.width() - 65, 8, ST77XX_BLACK);

  // Actual received coordinates
  tft.setCursor(65, 130);
  tft.print(latCS, 6);
  tft.setCursor(65, 145);
  tft.print(lonCS, 6);
}

//  ---------------------------------------------------------------
// ----------------------- CALCULATIONS ---------------------------
//  ---------------------------------------------------------------

void performMagnetometerCalibration() {
  Serial.println("\n=== MAGNETOMETER CALIBRATION ===");
  Serial.println("Rotate the device in all directions (figure-8) for 20s...");

  // Show calibration message on display
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(30, 10);
  tft.println("MAGNETOMETER");
  tft.setCursor(30, 30);
  tft.println("CALIBRATION");
  tft.setCursor(15, 50);
  tft.println("Rotate device in");
  tft.setCursor(15, 60);
  tft.println("all directions...");
  tft.setCursor(2, 90);
  tft.println("20 second calib start");

  unsigned long start = millis();
  int readCount = 0;
  unsigned long lastProgressUpdate = 0;

  // Calibration loop
  while (millis() - start < CAL_TIME) {
    int16_t x, y, z;

    if (readMagnetometerRaw(&x, &y, &z)) {
      readCount++;

      // Update min/max
      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;

      // Update progress every second
      if (millis() - lastProgressUpdate > 1000) {
        int remaining = (CAL_TIME - (millis() - start)) / 1000;
        tft.fillRect(5, 110, tft.width(), 10, ST77XX_BLACK);
        tft.setCursor(5, 110);
        tft.printf("%d seconds remaining", remaining);
        lastProgressUpdate = millis();
      }
    }
    delay(50);
  }

  if (readCount < 10) {
    Serial.println("ERROR: Too few reads during calibration!");
    magnetometerWorking = false;
    return;
  }

  // Compute offsets and scales
  xOffset = (xMax + xMin) / 2;
  yOffset = (yMax + yMin) / 2;
  zOffset = (zMax + zMin) / 2;

  double xRange = (double)(xMax - xMin);
  double yRange = (double)(yMax - yMin);
  double zRange = (double)(zMax - zMin);
  double avgRange = (xRange + yRange + zRange) / 3.0f;

  if (xRange > 0) xScale = avgRange / xRange;
  if (yRange > 0) yScale = avgRange / yRange;
  if (zRange > 0) zScale = avgRange / zRange;

  calibrationDone = true;

  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.printf("Offsets - X: %d, Y: %d, Z: %d\n", xOffset, yOffset, zOffset);
  Serial.printf("Scales - X: %.4f, Y: %.4f, Z: %.4f\n", xScale, yScale, zScale);

  // Show completion message
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(2, 40);
  tft.println("CALIBRATION COMPLETE!");
  tft.setCursor(25, 70);
  tft.println("System ready...");
  delay(2000);

  // Re-initialize display
  initializeDisplay();
}

// Bearing based on current and received GPS coordinates
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double deltaLon = radians(lon2 - lon1);

  double y = sin(deltaLon) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLon);

  double brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0)
    brng += 360;
  return brng;
}

// Distance based on current and received GPS coordinates
int calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double R = 6371000;
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double dPhi = radians(lat2 - lat1);
  double dLambda = radians(lon2 - lon1);

  double a = sin(dPhi / 2) * sin(dPhi / 2) + cos(phi1) * cos(phi2) * sin(dLambda / 2) * sin(dLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return (int)(R * c);
}

// Heading using magnetometer
double calculateMagneticHeading(int16_t mx, int16_t my) {
  if (mx == 0 && my == 0) return heading;

  // Read device orientation
  readDeviceOrientation();

  // Calculate raw magnetic heading
  double rawHeading = atan2(my, mx) * 180.0 / PI;

  // Apply magnetic declination for Hungary
  rawHeading += 2.5;

  // Normalize to 0-360
  if (rawHeading < 0) rawHeading += 360.0f;
  if (rawHeading >= 360.0f) rawHeading -= 360.0f;

  // Tilt compensation (simplified - for more accurate results, use full 3D rotation matrix)
  double compensatedHeading = rawHeading;

  // If device is tilted significantly, adjust heading calculation
  if (abs(deviceTiltX) > 15 || abs(deviceTiltY) > 15) {
    // This is a simplified compensation - for precise results you'd need
    // full 3D magnetometer calibration with tilt compensation
    Serial.printf("Device tilted: X=%.1f°, Y=%.1f° - heading may be inaccurate\n",
                  deviceTiltX, deviceTiltY);
  }

  return compensatedHeading;
}

// Get device orientation by accelerometer
void readDeviceOrientation() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculate tilt angles
  deviceTiltX = atan2(ay, az) * 180.0 / PI;
  deviceTiltY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}