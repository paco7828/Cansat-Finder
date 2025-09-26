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

// Instances
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
BetterGPS gps;
LoRaRx *loraRx = nullptr;  // Will be created with dynamic settings
BetterCapportal capportal;
MPU6050 mpu;
Preferences preferences;
LoRaConfig loraConfig;

// Track configuration mode
bool configMode = false;

// Calculation variables
float heading = 0;
float prevAngle = 0;
float angle = 0;
float latYou = 0, lonYou = 0;
float latCS = 0, lonCS = 0;

// Track valid received coordinates
bool validCSPos = false;

// Track magnetometer working status
bool magnetometerWorking = false;

// GPS heading fallback variables
float gpsHeading = 0;
bool hasGpsHeading = false;
unsigned long lastGpsUpdate = 0;

// Display variables
int meterValue;
bool wasShowingQuestion = false;
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
void drawStaticText();

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
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.println("CanSat Finder v2.0");
  tft.setCursor(10, 25);
  tft.println("Loading...");
  tft.setCursor(5, 40);
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
  tft.fillScreen(ST77XX_BLACK);
  drawStaticText();
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
    String receivedData = loraRx->listen();  // Use the new listen() method that returns data

    if (receivedData.length() > 0) {
      Serial.println("Processing received data: " + receivedData);

      int semicolonIndex = receivedData.indexOf(';');
      if (semicolonIndex != -1) {
        String latStr = receivedData.substring(0, semicolonIndex);
        String lonStr = receivedData.substring(semicolonIndex + 1);

        float newLatCS = latStr.toFloat();
        float newLonCS = lonStr.toFloat();

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

  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    if (gps.hasFix()) {
      latYou = gps.getLatitude();
      lonYou = gps.getLongitude();

      if (wasShowingQuestion) {
        clearQuestionMark();
        wasShowingQuestion = false;
      }

      // --- Determine heading source ---
      bool usingMagnetometer = false;

      if (magnetometerWorking) {
        // Try magnetometer first
        int16_t mx, my, mz;
        readMagnetometer(mx, my, mz);

        if (mx != 0 || my != 0) {
          heading = calculateMagneticHeading(mx, my);
          usingMagnetometer = true;
        }
      }

      if (!usingMagnetometer) {
        // Fallback to GPS heading when moving
        float currentSpeed = gps.getSpeedKmph();

        if (currentSpeed > MIN_SPEED_FOR_HEADING && gps.isCourseValid()) {
          gpsHeading = gps.getCourseDeg();
          hasGpsHeading = true;
          lastGpsUpdate = millis();
        }

        // Use GPS heading if available and recent
        if (hasGpsHeading && (millis() - lastGpsUpdate < 15000)) {
          heading = gpsHeading;
        } else {
          // Last resort: assume North
          heading = 0;
          hasGpsHeading = false;
        }
      }

      // --- GPS based cansat heading - ONLY if we have valid CS position ---
      if (validCSPos) {
        // Clear any previous "No CS" text first
        tft.fillRect(centerX - 25, centerY - 12, 50, 20, ST77XX_BLACK);

        angle = calculateBearing(latYou, lonYou, latCS, lonCS);

        // --- Compensation with own heading ---
        float arrowAngle = angle - heading;
        if (arrowAngle < 0)
          arrowAngle += 360;

        // Calculate distance
        meterValue = calculateDistance(latYou, lonYou, latCS, lonCS);

        // Clear old arrow
        drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);

        // Draw new arrow - color indicates heading source and CS data validity
        uint16_t arrowColor;
        if (usingMagnetometer) {
          arrowColor = ST77XX_GREEN;  // Green for magnetometer
        } else if (hasGpsHeading) {
          arrowColor = ST77XX_BLUE;  // Blue for GPS heading
        } else {
          arrowColor = ST77XX_YELLOW;  // Yellow for North assumption
        }

        drawRotatingArrow(centerX, centerY, 22, 30, arrowAngle, arrowColor);
        prevAngle = arrowAngle;
      } else {
        // No valid CS position - show a different indicator
        drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        tft.setCursor(centerX - 20, centerY - 8);
        tft.print("No CS");

        meterValue = 0;  // No distance calculation possible
      }

      // Update display
      drawCoordinates();
      drawCanSatCoordinates();
      drawMeter();

    } else {
      if (!wasShowingQuestion) {
        drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);
      }

      drawQuestionMark();
      wasShowingQuestion = true;

      drawCanSatCoordinates();  // Still show CanSat coords even without GPS
    }

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
  tft.println("Returning to normal...");

  delay(3000);
  tft.fillScreen(ST77XX_BLACK);
  drawStaticText();
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
  tft.println("CanSat-Config");
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

  // Set up captive portal
  capportal.setHTML(configHTML);
  capportal.onSubmit(handleConfigSubmission);

  // Start AP without password for easy access
  capportal.begin("SAS-MK3-finder", "sasmk3seat", 300000);  // 5 minutes timeout

  Serial.println("Configuration portal started!");
  Serial.println("Connect to WiFi: CanSat-Config");
  Serial.println("Then go to: http://4.3.2.1");
}

// Handle config button state
void checkConfigButton() {
  // Default variables
  static bool buttonPressed = false;
  static unsigned long pressStartTime = 0;
  bool currentState = !digitalRead(CONFIG_BUTTON_PIN);  // Active low

  if (currentState && !buttonPressed) {
    // Button just pressed
    buttonPressed = true;
    pressStartTime = millis();
  } else if (!currentState && buttonPressed) {
    // Button released
    buttonPressed = false;
    unsigned long pressDuration = millis() - pressStartTime;

    if (pressDuration >= CONFIG_BUTTON_HOLD_TIME) {
      // Long press detected => enter config mode if haven't already
      if (!configMode) {
        enterConfigMode();
      }
    }
  }

  // Show progress on display while button is held
  if (buttonPressed) {
    unsigned long currentDuration = millis() - pressStartTime;
    if (currentDuration >= CONFIG_BUTTON_HOLD_TIME && !configMode) {
      // Will enter config mode on release
    } else if (currentDuration > 1000) {
      // Show progress
      static unsigned long lastProgressUpdate = 0;
      if (millis() - lastProgressUpdate > 100) {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.fillRect(0, 150, tft.width(), 10, ST77XX_BLACK);
        tft.setCursor(5, 152);
        tft.printf("Hold for config: %d%%", (int)((currentDuration * 100) / CONFIG_BUTTON_HOLD_TIME));
        lastProgressUpdate = millis();
      }
    }
  }
}

//  ---------------------------------------------------------------
// -------------------- INITIALIZATION ----------------------------
//  ---------------------------------------------------------------

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

    // Configure QMC5883L
    // Mode: Continuous, ODR: 10Hz, Scale: 8G, OSR: 64
    writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG, 0x01);
    delay(10);

    // Set interrupt enable
    writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG2, 0x00);
    delay(10);

    // Verify configuration
    uint8_t config = readRegister(QMC5883L_ADDR, QMC5883L_CONFIG);
    Serial.print("QMC5883L Config register: 0x");
    Serial.println(config, HEX);

    if (config == 0x01) {
      magnetometerWorking = true;
      Serial.println("QMC5883L initialized successfully via MPU6050!");
    } else {
      Serial.println("QMC5883L configuration failed, using GPS heading");
      magnetometerWorking = false;
    }

  } else {
    Serial.println("MPU6050 connection failed, using GPS heading only");
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

// Read magnetometer
void readMagnetometer(int16_t &x, int16_t &y, int16_t &z) {
  // If magnetometer fails
  if (!magnetometerWorking) {
    // Set values to 0
    x = y = z = 0;
    return;
  }

  // Check if data ready
  uint8_t status = readRegister(QMC5883L_ADDR, QMC5883L_STATUS);
  if (!(status & 0x01)) {
    x = y = z = 0;
    return;
  }

  // Read 6 bytes of magnetometer data
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_X_LSB);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883L_ADDR, (uint8_t)6);
  if (Wire.available() >= 6) {
    x = Wire.read() | (Wire.read() << 8);
    y = Wire.read() | (Wire.read() << 8);
    z = Wire.read() | (Wire.read() << 8);
  } else {
    x = y = z = 0;
  }
}

//  ---------------------------------------------------------------
// ------------------------ DRAWINGS ------------------------------
//  ---------------------------------------------------------------

// Templates without values
void drawStaticText() {
  // Distance
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(centerX - 30, 5);
  tft.println("--- m");

  // Finder's coordinates template
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(0, 90);
  tft.print("Lat (you): ");
  tft.setCursor(0, 105);
  tft.print("Lon (you): ");

  // Separator line
  tft.drawLine(0, 120, tft.width(), 120, ST77XX_WHITE);

  // Received coordinates template
  tft.setCursor(0, 130);
  tft.print("Lat (CS): ");
  tft.setCursor(0, 145);
  tft.print("Lon (CS): ");
}

// Distance meter on display based on string length
void drawMeter() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Clear previous meter area
  tft.fillRect(0, 0, tft.width(), 20, ST77XX_BLACK);

  // Determine display string
  String displayStr;
  if (meterValue < 1000) {
    displayStr = String(meterValue) + " m";
  } else {
    float km = meterValue / 1000.0;
    displayStr = String(km, 2) + " km";
  }

  // Measure text width and center it
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(displayStr, 0, 0, &x1, &y1, &w, &h);
  int16_t xPos = (tft.width() - w) / 2;

  // Actual meter value
  tft.setCursor(xPos, 0);
  tft.print(displayStr);
}


// Clear '?' from display
void clearQuestionMark() {
  tft.setTextSize(4);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.fillRect(centerX - charWidth / 2, centerY - charHeight / 2, charWidth, charHeight, ST77XX_BLACK);
}

// Draw '?' on display
void drawQuestionMark() {
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.setCursor(centerX - charWidth / 2, centerY - charHeight / 2);
  tft.print("?");
}

// Arrow pointing to received coordinates
void drawRotatingArrow(int cx, int cy, int headSize, int stickLen, float angleDeg, uint16_t color) {
  float rad = radians(angleDeg);

  int x0 = 0, y0 = -headSize;
  int x1 = -headSize / 2, y1 = 8;
  int x2 = headSize / 2, y2 = 8;

  int stickW = 6;
  int sx = -stickW / 2, sy = 8;
  int ex = stickW / 2, ey = stickLen;

  auto rotX = [&](int x, int y) {
    return (int)(x * cos(rad) - y * sin(rad)) + cx;
  };
  auto rotY = [&](int x, int y) {
    return (int)(x * sin(rad) + y * cos(rad)) + cy;
  };

  tft.fillTriangle(rotX(x0, y0), rotY(x0, y0),
                   rotX(x1, y1), rotY(x1, y1),
                   rotX(x2, y2), rotY(x2, y2),
                   color);

  tft.fillTriangle(rotX(sx, sy), rotY(sx, sy),
                   rotX(ex, sy), rotY(ex, sy),
                   rotX(sx, ey), rotY(sx, ey),
                   color);
  tft.fillTriangle(rotX(ex, sy), rotY(ex, sy),
                   rotX(ex, ey), rotY(ex, ey),
                   rotX(sx, ey), rotY(sx, ey),
                   color);
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

// Bearing based on current and received GPS coordinates
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaLon = radians(lon2 - lon1);

  float y = sin(deltaLon) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLon);

  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0)
    brng += 360;
  return brng;
}

// Distance based on current and received GPS coordinates
int calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float dPhi = radians(lat2 - lat1);
  float dLambda = radians(lon2 - lon1);

  float a = sin(dPhi / 2) * sin(dPhi / 2) + cos(phi1) * cos(phi2) * sin(dLambda / 2) * sin(dLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return (int)(R * c);
}

// Heading using magnetometer
float calculateMagneticHeading(int16_t mx, int16_t my) {
  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}