#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "Better-GPS.h"
#include <math.h>

#define TFT_CS 5
#define TFT_DC 1
#define TFT_RST 0
#define TFT_SCLK 6
#define TFT_MOSI 7
#define GPS_RX 2

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
BetterGPS gps;

float angle = 0;
float latYou = 0, lonYou = 0, latCS = 0, lonCS = 0;
int meterValue;
bool wasShowingQuestion = false;  // track if we were showing "?"

int centerX;
int centerY = 55;  // arrow center

void setup() {
  randomSeed(analogRead(A0));
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  gps.begin(GPS_RX);

  centerX = tft.width() / 2;

  // draw fixed texts once
  drawStaticText();
}

float prevAngle = 0;

void loop() {
  gps.update();

  if (gps.hasFix()) {
    latYou = gps.getLatitude();
    lonYou = gps.getLongitude();

    // Update
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 100) {
      // Generate random coordinates 500-2000 meters from current location
      generateRandomNearbyCoordinates(latYou, lonYou, 500, 2000);
      drawCanSatCoordinates();  // Update CS coordinates on screen
      lastUpdate = millis();
    }

    // If we were showing "?", clear it first
    if (wasShowingQuestion) {
      clearQuestionMark();
      wasShowingQuestion = false;
    }

    // calculate angle to CS
    if (latCS != 0 && lonCS != 0) {  // Only if we have CS coordinates
      angle = calculateBearing(latYou, lonYou, latCS, lonCS);
      meterValue = calculateDistance(latYou, lonYou, latCS, lonCS);
    } else {
      // Keep spinning even without CS coordinates
      angle += 5;  // Increment by 5 degrees each loop
      if (angle >= 360) angle = 0;
    }

    // erase old arrow
    drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);

    // draw new arrow
    drawRotatingArrow(centerX, centerY, 22, 30, angle, ST77XX_GREEN);

    prevAngle = angle;  // save current for next erase

    // Only update coordinates and distance when we have a GPS fix
    drawCoordinates();
    drawMeter();

  } else {
    // No GPS fix

    // Show "?" only
    if (!wasShowingQuestion) {
      // erase old arrow first
      drawRotatingArrow(centerX, centerY, 22, 30, prevAngle, ST77XX_BLACK);
    }

    drawQuestionMark();
    wasShowingQuestion = true;

  }
}

// Generate random coordinates within specified distance range from current position
void generateRandomNearbyCoordinates(float currentLat, float currentLon, int minDistanceM, int maxDistanceM) {
  // Generate random distance between min and max
  int distance = random(minDistanceM, maxDistanceM + 1);

  // Generate random bearing (0-360 degrees)
  float bearing = random(0, 360);

  // Convert to radians
  float bearingRad = radians(bearing);

  // Earth's radius in meters
  float R = 6371000.0;

  // Convert current position to radians
  float lat1Rad = radians(currentLat);
  float lon1Rad = radians(currentLon);

  // Calculate new position using spherical geometry
  float lat2Rad = asin(sin(lat1Rad) * cos(distance / R) + cos(lat1Rad) * sin(distance / R) * cos(bearingRad));

  float lon2Rad = lon1Rad + atan2(sin(bearingRad) * sin(distance / R) * cos(lat1Rad), cos(distance / R) - sin(lat1Rad) * sin(lat2Rad));

  // Convert back to degrees
  latCS = degrees(lat2Rad);
  lonCS = degrees(lon2Rad);

  // Normalize longitude to -180 to 180 range
  if (lonCS > 180) lonCS -= 360;
  if (lonCS < -180) lonCS += 360;
}

// Clear the question mark
void clearQuestionMark() {
  tft.setTextSize(4);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.fillRect(centerX - charWidth / 2, centerY - charHeight / 2, charWidth, charHeight, ST77XX_BLACK);
}

// Draw question mark
void drawQuestionMark() {
  tft.setTextSize(4);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  int charWidth = 6 * 4;
  int charHeight = 8 * 4;
  tft.setCursor(centerX - charWidth / 2, centerY - charHeight / 2);
  tft.print("?");
}

// static labels (draw only once)
void drawStaticText() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(centerX - 30, 5);
  tft.println("0 m");  // placeholder

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

// Update CanSat coordinates display
void drawCanSatCoordinates() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Clear old coordinates first
  tft.fillRect(65, 130, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 145, tft.width() - 65, 8, ST77XX_BLACK);

  // Draw new coordinates
  tft.setCursor(65, 130);
  tft.print(latCS, 7);
  tft.setCursor(65, 145);
  tft.print(lonCS, 7);
}

// overwrite only user coordinates
void drawCoordinates() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Clear old coordinates first
  tft.fillRect(65, 90, tft.width() - 65, 8, ST77XX_BLACK);
  tft.fillRect(65, 105, tft.width() - 65, 8, ST77XX_BLACK);

  tft.setCursor(65, 90);
  tft.print(latYou, 7);
  tft.setCursor(65, 105);
  tft.print(lonYou, 7);
}

// draw distance
void drawMeter() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // Clear old distance first - wider rectangle to handle longer numbers
  tft.fillRect(centerX - 40, 5, 80, 16, ST77XX_BLACK);

  tft.setCursor(centerX - 30, 5);
  tft.print(meterValue);
  tft.print(" m");
}

// arrow drawing (same as before)
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

// calculate bearing in degrees from user to CS
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaLon = radians(lon2 - lon1);

  float y = sin(deltaLon) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLon);

  float brng = atan2(y, x);
  brng = degrees(brng);
  if (brng < 0) brng += 360;
  return brng;
}

// calculate distance in meters
int calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371000;  // Earth radius in meters
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float dPhi = radians(lat2 - lat1);
  float dLambda = radians(lon2 - lon1);

  float a = sin(dPhi / 2) * sin(dPhi / 2) + cos(phi1) * cos(phi2) * sin(dLambda / 2) * sin(dLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return (int)(R * c);
}