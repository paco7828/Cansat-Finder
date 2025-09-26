/* Improved QMC5883L example for ESP32-WROOM-DA
   - Auto calibration (min/max) for a limited time
   - Applies offsets and optional scaling
   - Averages multiple samples and low-pass filters the heading
   - Uses MechaQMC5883L library (MechaQMC5883.h)
*/

#include <Wire.h>
#include <MechaQMC5883.h>

MechaQMC5883 qmc;

// --- Config ---
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const unsigned long CAL_TIME = 20000UL;  // rotate for 20 seconds during startup
const int N_SAMPLES = 8;                 // number of raw samples to average each loop
const float HEADING_ALPHA = 0.08f;       // low-pass alpha (0..1), lower = smoother

// --- Calibration containers (signed because raw values are signed)
int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
int16_t zMin = 32767, zMax = -32768;

// offsets & scales computed after calibration
int16_t xOffset = 0, yOffset = 0, zOffset = 0;
float xScale = 1.0f, yScale = 1.0f, zScale = 1.0f;

float filteredHeading = -1.0f;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);

  qmc.init();  // just initialize, no return value
  Serial.println("\nQMC5883L starting...");
  Serial.println("=== CALIBRATION PHASE ===");
  Serial.println("Rotate the module in all directions (figure-8) for 20s...");

  unsigned long start = millis();

  // Calibration loop - collect min/max
  while (millis() - start < CAL_TIME) {
    uint16_t xr, yr, zr;
    qmc.read(&xr, &yr, &zr);
    int16_t x = (int16_t)xr;
    int16_t y = (int16_t)yr;
    int16_t z = (int16_t)zr;

    // update min/max
    if (x < xMin) xMin = x;
    if (x > xMax) xMax = x;
    if (y < yMin) yMin = y;
    if (y > yMax) yMax = y;
    if (z < zMin) zMin = z;
    if (z > zMax) zMax = z;

    // show live raw values occasionally
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 800) {
      Serial.print("raw: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      lastPrint = millis();
    }
    delay(50);
  }

  // compute offsets (hard-iron)
  xOffset = (xMax + xMin) / 2;
  yOffset = (yMax + yMin) / 2;
  zOffset = (zMax + zMin) / 2;

  // optional soft-iron scale correction to make ranges equal
  float xRange = (float)(xMax - xMin);
  float yRange = (float)(yMax - yMin);
  float zRange = (float)(zMax - zMin);
  float avgRange = (xRange + yRange + zRange) / 3.0f;
  if (xRange > 0) xScale = avgRange / xRange;
  if (yRange > 0) yScale = avgRange / yRange;
  if (zRange > 0) zScale = avgRange / zRange;

  Serial.println("\n=== CALIBRATION RESULT ===");
  Serial.print("xMin=");
  Serial.print(xMin);
  Serial.print("  xMax=");
  Serial.print(xMax);
  Serial.print("  xOffset=");
  Serial.println(xOffset);
  Serial.print("yMin=");
  Serial.print(yMin);
  Serial.print("  yMax=");
  Serial.print(yMax);
  Serial.print("  yOffset=");
  Serial.println(yOffset);
  Serial.print("zMin=");
  Serial.print(zMin);
  Serial.print("  zMax=");
  Serial.print(zMax);
  Serial.print("  zOffset=");
  Serial.println(zOffset);
  Serial.print("xScale=");
  Serial.print(xScale, 4);
  Serial.print("  yScale=");
  Serial.print(yScale, 4);
  Serial.print("  zScale=");
  Serial.println(zScale, 4);
  Serial.println("You can copy these offsets/scales into your code to skip calibration next time.");
  Serial.println("============================\n");
}

void loop() {
  // average several raw samples to reduce noise
  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < N_SAMPLES; ++i) {
    uint16_t xr, yr, zr;
    qmc.read(&xr, &yr, &zr);
    int16_t x = (int16_t)xr;
    int16_t y = (int16_t)yr;
    int16_t z = (int16_t)zr;
    sx += (long)x;
    sy += (long)y;
    sz += (long)z;
    delay(6);  // small spacing
  }
  float xAvg = (float)sx / (float)N_SAMPLES;
  float yAvg = (float)sy / (float)N_SAMPLES;
  float zAvg = (float)sz / (float)N_SAMPLES;

  // apply offsets and scales (hard + soft iron correction)
  float xCal = (xAvg - (float)xOffset) * xScale;
  float yCal = (yAvg - (float)yOffset) * yScale;
  float zCal = (zAvg - (float)zOffset) * zScale;

  // heading from x/y (2D). If you want tilt-compensated heading, use accel data.
  float heading = atan2(yCal, xCal) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;

  // small low-pass on heading to avoid jumps (handles wrap-around)
  if (filteredHeading < 0) {
    filteredHeading = heading;  // init
  } else {
    // handle wrap-around properly: compute shortest delta
    float diff = heading - filteredHeading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    filteredHeading += HEADING_ALPHA * diff;
    // keep in 0..360
    if (filteredHeading < 0) filteredHeading += 360.0f;
    if (filteredHeading >= 360.0f) filteredHeading -= 360.0f;
  }

  // print results
  Serial.print("RawAvg X: ");
  Serial.print((int)xAvg);
  Serial.print(" Y: ");
  Serial.print((int)yAvg);
  Serial.print(" | Cal X: ");
  Serial.print((int)round(xCal));
  Serial.print(" Y: ");
  Serial.print((int)round(yCal));
  Serial.print(" | Heading: ");
  Serial.print(filteredHeading, 1);
  Serial.println("°");

  delay(120);
}
