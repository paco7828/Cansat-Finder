#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "Better-GPS.h"
#include "LoRaRx.h"
#include <math.h>

#define TFT_CS 5
#define TFT_DC 2
#define TFT_RST 4
#define TFT_SCLK 18
#define TFT_MOSI 23
#define GPS_RX 13

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
BetterGPS gps;
LoRaRx loraRx;

float angle = 0;
float latYou = 0, lonYou = 0;

// Dynamic CanSat coordinates from LoRa
float latCS = 0;
float lonCS = 0;
bool validCSPos = false;

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

  // Start LoRa
  loraRx.begin();

  // Draw fixed display elements
  drawStaticText();
  Serial.println("Display initialized. System ready.");
}

float prevAngle = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 200;

void loop() {
  unsigned long currentTime = millis();

  // Update GPS data
  gps.update();

  // Listen for LoRa data
  loraRx.listen(false);

  // Check if any LoRa message received
  if (Serial2.available()) {
    String radioData = Serial2.readStringUntil('\r');
    radioData.trim();
    if (radioData.length() > 0) {
      // Expecting "lat;lon"
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

  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {

    if (gps.hasFix()) {
      latYou = gps.getLatitude();
      lonYou = gps.getLongitude();

      if (wasShowingQuestion) {
        clearQuestionMark();
        wasShowingQuestion = false;
      }

      // Calculate angle and distance to CanSat
      angle = calculateBearing(latYou, lonYou, latCS, lonCS);
      meterValue = calculateDistance(latYou, lonYou, latCS, lonCS);

      // Clear old arrow
      drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);

      // Draw new arrow
      drawRotatingArrow(centerX, centerY, 22, 30, angle, ST77XX_GREEN);
      prevAngle = angle;

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
      Serial.printf("GPS Fix: %.6f, %.6f\n", latYou, lonYou);
    } else {
      Serial.println("GPS: No fix");
    }

    Serial.printf("CanSat: %.6f, %.6f\n", latCS, lonCS);

    lastStatusPrint = currentTime;
  }

  delay(50);
}

// --- Display helper functions remain unchanged ---

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
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
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
