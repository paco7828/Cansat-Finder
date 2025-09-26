#include <Wire.h>

// QMC5883L I2C address and registers
#define QMC5883L_ADDR 0x0D
#define QMC5883L_X_LSB 0x00
#define QMC5883L_X_MSB 0x01
#define QMC5883L_Y_LSB 0x02
#define QMC5883L_Y_MSB 0x03
#define QMC5883L_Z_LSB 0x04
#define QMC5883L_Z_MSB 0x05
#define QMC5883L_STATUS 0x06
#define QMC5883L_TEMP_LSB 0x07
#define QMC5883L_TEMP_MSB 0x08
#define QMC5883L_CONFIG 0x09
#define QMC5883L_CONFIG2 0x0A
#define QMC5883L_RESET 0x0B
#define QMC5883L_RESERVED 0x0C
#define QMC5883L_CHIP_ID 0x0D

// MPU6050 registers
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_USER_CTRL 0x6A

// --- Config ---
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const unsigned long CAL_TIME = 20000UL;  // rotate for 20 seconds during startup
const int N_SAMPLES = 5;                 // number of raw samples to average each loop
const float HEADING_ALPHA = 0.08f;       // low-pass alpha (0..1), lower = smoother

// --- Calibration containers
int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
int16_t zMin = 32767, zMax = -32768;

// offsets & scales
int16_t xOffset = 0, yOffset = 0, zOffset = 0;
float xScale = 1.0f, yScale = 1.0f, zScale = 1.0f;

float filteredHeading = -1.0f;
bool sensorInitialized = false;

void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  uint8_t error = Wire.endTransmission();
  if (error != 0) {
    Serial.print("I2C write error: ");
    Serial.print(error);
    Serial.print(" to addr 0x");
    Serial.print(addr, HEX);
    Serial.print(" reg 0x");
    Serial.println(reg, HEX);
  }
  delay(10);
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return 0xFF;  // Error indicator
  }

  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;  // Error indicator
}

bool readMagnetometer(int16_t* x, int16_t* y, int16_t* z) {
  // Check if data is ready
  uint8_t status = readRegister(QMC5883L_ADDR, QMC5883L_STATUS);
  if (status == 0xFF || !(status & 0x01)) {
    return false;  // Data not ready or read error
  }

  // Read all 6 bytes at once
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

bool initQMC5883L() {
  Serial.println("Initializing QMC5883L...");

  // Reset the chip
  writeRegister(QMC5883L_ADDR, QMC5883L_RESET, 0x01);
  delay(100);

  // Check chip ID (should be 0xFF)
  uint8_t chipId = readRegister(QMC5883L_ADDR, QMC5883L_CHIP_ID);
  Serial.print("QMC5883L Chip ID: 0x");
  Serial.println(chipId, HEX);

  // Configure QMC5883L
  // MODE: Continuous, ODR: 200Hz, RNG: 2G, OSR: 64
  writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG, 0x1D);

  // Set interrupt enable
  writeRegister(QMC5883L_ADDR, QMC5883L_CONFIG2, 0x00);

  delay(100);

  // Test read
  int16_t x, y, z;
  for (int i = 0; i < 10; i++) {
    if (readMagnetometer(&x, &y, &z)) {
      Serial.print("Test read successful: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.println(z);
      return true;
    }
    delay(100);
  }

  Serial.println("Failed to read from QMC5883L");
  return false;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz for better reliability
  delay(100);

  Serial.println("GY-87 Magnetometer Test Starting...");

  // --- Configure MPU6050 to enable bypass mode
  Serial.println("Configuring MPU6050...");

  // Wake up MPU6050
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(100);

  // Disable I2C master mode and enable bypass
  writeRegister(MPU6050_ADDR, MPU6050_USER_CTRL, 0x00);
  delay(10);

  // Enable I2C bypass
  writeRegister(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02);
  delay(100);

  Serial.println("MPU6050 bypass mode enabled");

  // --- Initialize QMC5883L
  sensorInitialized = initQMC5883L();

  if (!sensorInitialized) {
    Serial.println("ERROR: Could not initialize QMC5883L!");
    Serial.println("Check your wiring and connections.");
    return;
  }

  Serial.println("\n=== CALIBRATION PHASE ===");
  Serial.println("Rotate the module in all directions (figure-8) for 20s...");

  unsigned long start = millis();
  int readCount = 0;

  // Calibration loop - collect min/max
  while (millis() - start < CAL_TIME) {
    int16_t x, y, z;

    if (readMagnetometer(&x, &y, &z)) {
      readCount++;

      // update min/max
      if (x < xMin) xMin = x;
      if (x > xMax) xMax = x;
      if (y < yMin) yMin = y;
      if (y > yMax) yMax = y;
      if (z < zMin) zMin = z;
      if (z > zMax) zMax = z;

      static unsigned long lastPrint = 0;
      if (millis() - lastPrint > 1000) {
        Serial.print("Raw: ");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(", ");
        Serial.print(z);
        Serial.print(" | Reads: ");
        Serial.println(readCount);
        lastPrint = millis();
      }
    }
    delay(50);
  }

  if (readCount < 10) {
    Serial.println("ERROR: Too few successful reads during calibration!");
    sensorInitialized = false;
    return;
  }

  // Compute offsets and scales (hard + soft iron correction)
  xOffset = (xMax + xMin) / 2;
  yOffset = (yMax + yMin) / 2;
  zOffset = (zMax + zMin) / 2;

  float xRange = (float)(xMax - xMin);
  float yRange = (float)(yMax - yMin);
  float zRange = (float)(zMax - zMin);
  float avgRange = (xRange + yRange + zRange) / 3.0f;

  if (xRange > 0) xScale = avgRange / xRange;
  if (yRange > 0) yScale = avgRange / yRange;
  if (zRange > 0) zScale = avgRange / zRange;

  Serial.println("\n=== CALIBRATION RESULT ===");
  Serial.print("Ranges - X: ");
  Serial.print(xRange);
  Serial.print(", Y: ");
  Serial.print(yRange);
  Serial.print(", Z: ");
  Serial.println(zRange);
  Serial.print("Offsets - X: ");
  Serial.print(xOffset);
  Serial.print(", Y: ");
  Serial.print(yOffset);
  Serial.print(", Z: ");
  Serial.println(zOffset);
  Serial.print("Scales - X: ");
  Serial.print(xScale, 4);
  Serial.print(", Y: ");
  Serial.print(yScale, 4);
  Serial.print(", Z: ");
  Serial.println(zScale, 4);
  Serial.println("============================\n");
}

void loop() {
  if (!sensorInitialized) {
    Serial.println("Sensor not initialized. Retrying...");
    sensorInitialized = initQMC5883L();
    delay(5000);
    return;
  }

  long sx = 0, sy = 0, sz = 0;
  int successfulReads = 0;

  for (int i = 0; i < N_SAMPLES; ++i) {
    int16_t x, y, z;
    if (readMagnetometer(&x, &y, &z)) {
      sx += (long)x;
      sy += (long)y;
      sz += (long)z;
      successfulReads++;
    }
    delay(20);
  }

  if (successfulReads == 0) {
    Serial.println("No successful magnetometer reads!");
    delay(500);
    return;
  }

  float xAvg = (float)sx / (float)successfulReads;
  float yAvg = (float)sy / (float)successfulReads;
  float zAvg = (float)sz / (float)successfulReads;

  float xCal = (xAvg - (float)xOffset) * xScale;
  float yCal = (yAvg - (float)yOffset) * yScale;
  float zCal = (zAvg - (float)zOffset) * zScale;

  float heading = atan2(yCal, xCal) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;

  if (filteredHeading < 0) {
    filteredHeading = heading;
  } else {
    float diff = heading - filteredHeading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    filteredHeading += HEADING_ALPHA * diff;
    if (filteredHeading < 0) filteredHeading += 360.0f;
    if (filteredHeading >= 360.0f) filteredHeading -= 360.0f;
  }

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
  Serial.print("° (");
  Serial.print(successfulReads);
  Serial.println(" samples)");

  delay(200);
}