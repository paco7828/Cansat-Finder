#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define TFT_CS 5
#define TFT_DC 1
#define TFT_RST 0
#define TFT_SCLK 6
#define TFT_MOSI 7

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

float angle = 0;  // rotation angle
float latYou, lonYou, latCS, lonCS;
int meterValue;

int centerX;
int centerY = 55;  // arrow center

void setup() {
  randomSeed(analogRead(A0));
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  centerX = tft.width() / 2;

  // draw fixed texts once
  drawStaticText();
}

void loop() {
  // erase old arrow
  drawRotatingArrow(centerX, centerY, 22, 30, angle, ST77XX_BLACK);

  // rotate
  angle += 15;
  if (angle >= 360) angle = 0;

  // draw new arrow
  drawRotatingArrow(centerX, centerY, 22, 30, angle, ST77XX_GREEN);

  // random coords
  updateCoordinates();
  drawCoordinates();

  // update meter
  updateMeter();
  drawMeter();
}

// static labels (draw only once)
void drawStaticText() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(centerX - 30, 5);
  tft.println("300 m");  // initial placeholder

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

// random 7-digit coords
void updateCoordinates() {
  latYou = random(440000000, 450000000) / 10000000.0;
  lonYou = random(150000000, 160000000) / 10000000.0;
  latCS = random(460000000, 470000000) / 10000000.0;
  lonCS = random(180000000, 190000000) / 10000000.0;
}

// overwrite only values, 7 digits
void drawCoordinates() {
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setCursor(65, 90);
  tft.print(latYou, 7);
  tft.setCursor(65, 105);
  tft.print(lonYou, 7);
  tft.setCursor(65, 130);
  tft.print(latCS, 7);
  tft.setCursor(65, 145);
  tft.print(lonCS, 7);
}

// random meter
void updateMeter() {
  meterValue = random(100, 500);  // random between 100 and 500 meters
}

// draw meter
void drawMeter() {
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);  // overwrite old value
  tft.setCursor(centerX - 30, 5);
  tft.print(meterValue);
  tft.print(" m");
}

// arrow drawing
void drawRotatingArrow(int cx, int cy, int headSize, int stickLen, float angleDeg, uint16_t color) {
  float rad = radians(angleDeg);

  // arrow head
  int x0 = 0, y0 = -headSize;
  int x1 = -headSize / 2, y1 = 8;
  int x2 = headSize / 2, y2 = 8;

  // stick (wide)
  int stickW = 6;
  int sx = -stickW / 2, sy = 8;
  int ex = stickW / 2, ey = stickLen;

  auto rotX = [&](int x, int y) {
    return (int)(x * cos(rad) - y * sin(rad)) + cx;
  };
  auto rotY = [&](int x, int y) {
    return (int)(x * sin(rad) + y * cos(rad)) + cy;
  };

  // head
  tft.fillTriangle(rotX(x0, y0), rotY(x0, y0),
                   rotX(x1, y1), rotY(x1, y1),
                   rotX(x2, y2), rotY(x2, y2),
                   color);

  // stick
  tft.fillTriangle(rotX(sx, sy), rotY(sx, sy),
                   rotX(ex, sy), rotY(ex, sy),
                   rotX(sx, ey), rotY(sx, ey),
                   color);
  tft.fillTriangle(rotX(ex, sy), rotY(ex, sy),
                   rotX(ex, ey), rotY(ex, ey),
                   rotX(sx, ey), rotY(sx, ey),
                   color);
}
