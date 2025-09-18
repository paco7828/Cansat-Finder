#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "Better-GPS.h"
#include "LoRaRx.h"
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define TFT_CS 5
#define TFT_DC 2
#define TFT_RST 4
#define TFT_SCLK 18
#define TFT_MOSI 23
#define GPS_RX 13

// QMC5883L registers and constants
#define QMC5883L_ADDR 0x0D
#define QMC5883L_X_LSB 0x00
#define QMC5883L_X_MSB 0x01
#define QMC5883L_Y_LSB 0x02
#define QMC5883L_Y_MSB 0x03
#define QMC5883L_Z_LSB 0x04
#define QMC5883L_Z_MSB 0x05
#define QMC5883L_STATUS 0x06
#define QMC5883L_CONFIG 0x09
#define QMC5883L_CONFIG2 0x0A
#define QMC5883L_RESET 0x0B

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
BetterGPS gps;
LoRaRx loraRx;
MPU6050 mpu;

float heading = 0;
float angle = 0;
float latYou = 0, lonYou = 0;
float latCS = 0, lonCS = 0;
bool validCSPos = false;
bool magnetometerWorking = false;

// GPS heading fallback variables
float gpsHeading = 0;
bool hasGpsHeading = false;
unsigned long lastGpsUpdate = 0;
const float MIN_SPEED_FOR_HEADING = 2.0;  // km/h minimum speed for reliable GPS heading

// SHOWCASE mode variables
bool showcaseMode = false;
unsigned long lastShowcaseUpdate = 0;
const unsigned long SHOWCASE_UPDATE_INTERVAL = 500;  // Update every 500 ms
float showcaseBaseRadius = 500.0;                     // Base radius in meters
float showcaseCurrentRadius = 0;
float showcaseAngle = 0;
float showcaseSpeed = 45.0;  // degrees per update

int meterValue;
bool wasShowingQuestion = false;

int centerX;
int centerY = 55;  // arrow center

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== CanSat Finder Starting ===");

  // Initialize display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  centerX = tft.width() / 2;

  // Initialize GPS
  Serial.println("Initializing GPS...");
  gps.begin(GPS_RX);
  Serial.println("GPS initialized on pin " + String(GPS_RX));

  // Initialize LoRa
  loraRx.begin();

  // Initialize I2C & magnetometer
  Wire.begin(21, 22);
  initMagnetometer();

  // Draw fixed display elements
  drawStaticText();
  Serial.println("Display initialized. System ready.");

  // SHOWCASE mode can be enabled by calling startShowcase()
  // Uncomment the line below to enable showcase mode on startup
  startShowcase();
}

// SHOWCASE FUNCTIONS
void startShowcase() {
  showcaseMode = true;
  showcaseCurrentRadius = showcaseBaseRadius;
  showcaseAngle = 0;
  lastShowcaseUpdate = 0;
  Serial.println("SHOWCASE mode activated - generating random CanSat coordinates");
}

void stopShowcase() {
  showcaseMode = false;
  Serial.println("SHOWCASE mode deactivated - using LoRa coordinates");
}

void updateShowcaseCoordinates() {
  unsigned long currentTime = millis();

  if (!showcaseMode || !gps.hasFix()) {
    return;
  }

  if (currentTime - lastShowcaseUpdate >= SHOWCASE_UPDATE_INTERVAL) {
    // Get current GPS position as center
    float centerLat = gps.getLatitude();
    float centerLon = gps.getLongitude();

    // Create circular motion with varying radius
    showcaseAngle += showcaseSpeed;
    if (showcaseAngle >= 360) {
      showcaseAngle -= 360;
    }

    // Vary the radius over time (creates spiral-like effect)
    showcaseCurrentRadius = showcaseBaseRadius + (showcaseBaseRadius * 0.5 * sin(radians(showcaseAngle * 0.5)));

    // Convert polar coordinates to lat/lon offset
    // Approximate conversion: 1 degree lat ≈ 111,320 meters
    // 1 degree lon ≈ 111,320 * cos(latitude) meters
    float latOffset = (showcaseCurrentRadius * cos(radians(showcaseAngle))) / 111320.0;
    float lonOffset = (showcaseCurrentRadius * sin(radians(showcaseAngle))) / (111320.0 * cos(radians(centerLat)));

    // Update CanSat coordinates
    latCS = centerLat + latOffset;
    lonCS = centerLon + lonOffset;

    Serial.printf("SHOWCASE: Generated CanSat coords: %.6f, %.6f (radius: %.0fm, angle: %.0f°)\n",
                  latCS, lonCS, showcaseCurrentRadius, showcaseAngle);

    lastShowcaseUpdate = currentTime;
  }
}

void initMagnetometer() {
  Serial.println("Initializing magnetometer system...");

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

void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void readMagnetometer(int16_t &x, int16_t &y, int16_t &z) {
  if (!magnetometerWorking) {
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

float calculateMagneticHeading(int16_t mx, int16_t my) {
  float heading = atan2(my, mx) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

float prevAngle = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 200;

void loop() {
  unsigned long currentTime = millis();

  // Update GPS data
  gps.update();

  // Update coordinates based on mode
  if (showcaseMode) {
    updateShowcaseCoordinates();
  } else {
    // Listen for LoRa data only if not in showcase mode
    loraRx.listen(false);

    // Check if any LoRa message received
    if (Serial2.available()) {
      String radioData = Serial2.readStringUntil('\r');
      radioData.trim();
      if (radioData.length() > 0) {
        int sepIndex = radioData.indexOf(';');
        if (sepIndex > 0) {
          String latStr = radioData.substring(0, sepIndex);
          String lonStr = radioData.substring(sepIndex + 1);
          latCS = latStr.toFloat();
          lonCS = lonStr.toFloat();
          Serial.printf("Updated CanSat coordinates: %.6f, %.6f\n", latCS, lonCS);
        }
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

      // --- GPS based cansat heading ---
      angle = calculateBearing(latYou, lonYou, latCS, lonCS);

      // --- Compensation with own heading ---
      float arrowAngle = angle - heading;
      if (arrowAngle < 0)
        arrowAngle += 360;

      // Calculate distance
      meterValue = calculateDistance(latYou, lonYou, latCS, lonCS);

      // Clear old arrow
      drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);

      // Draw new arrow - color indicates heading source and mode
      uint16_t arrowColor;
      if (showcaseMode) {
        arrowColor = ST77XX_MAGENTA;  // Magenta for showcase mode
      } else if (usingMagnetometer) {
        arrowColor = ST77XX_GREEN;  // Green for magnetometer
      } else if (hasGpsHeading) {
        arrowColor = ST77XX_BLUE;  // Blue for GPS heading
      } else {
        arrowColor = ST77XX_YELLOW;  // Yellow for North assumption
      }

      drawRotatingArrow(centerX, centerY, 22, 30, arrowAngle, arrowColor);
      prevAngle = arrowAngle;

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

    Serial.printf("CanSat: %.6f, %.6f %s\n", latCS, lonCS, showcaseMode ? "(SHOWCASE)" : "(LoRa)");

    // Print heading info
    if (magnetometerWorking) {
      int16_t mx, my, mz;
      readMagnetometer(mx, my, mz);
      Serial.printf("Magnetometer: X=%d, Y=%d, Z=%d, Heading=%.1f°\n", mx, my, mz, heading);
    } else {
      Serial.printf("Heading source: %s (%.1f°)\n", hasGpsHeading ? "GPS" : "North", heading);
    }

    lastStatusPrint = currentTime;
  }

  delay(50);
}

void clearQuestionMark() {
  tft.setTextSize(4);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.fillRect(centerX - charWidth / 2, centerY - charHeight / 2, charWidth, charHeight, ST77XX_BLACK);
}

void drawQuestionMark() {
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.setCursor(centerX - charWidth / 2, centerY - charHeight / 2);
  tft.print("?");
}

void drawStaticText() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(centerX - 30, 5);
  tft.println("--- m");

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setCursor(0, 90);
  tft.print("Lat (you): ");
  tft.setCursor(0, 105);
  tft.print("Lon (you): ");
  tft.drawLine(0, 120, tft.width(), 120, ST77XX_WHITE);
  tft.setCursor(0, 130);
  tft.print("Lat (CS): ");
  tft.setCursor(0, 145);
  tft.print("Lon (CS): ");
}

void drawCanSatCoordinates() {
  tft.setTextSize(1);

  // Change color based on mode - magenta for showcase, green for LoRa
  uint16_t coordColor = showcaseMode ? ST77XX_MAGENTA : ST77XX_GREEN;
  tft.setTextColor(coordColor, ST77XX_BLACK);

  tft.fillRect(65, 130, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 145, tft.width() - 65, 8, ST77XX_BLACK);

  tft.setCursor(65, 130);
  tft.print(latCS, 6);
  tft.setCursor(65, 145);
  tft.print(lonCS, 6);
}

void drawCoordinates() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.fillRect(65, 90, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 105, tft.width() - 65, 8, ST77XX_BLACK);

  if (gps.hasFix()) {
    tft.setCursor(65, 90);
    tft.print(latYou, 6);
    tft.setCursor(65, 105);
    tft.print(lonYou, 6);
  }
}

void drawMeter() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Clear previous meter area (make it slightly wider)
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

  tft.setCursor(xPos, 0);
  tft.print(displayStr);
}

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