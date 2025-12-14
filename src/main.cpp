// Libs & Headers
#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include "Better-GPS.h"
#include "LoRaRx.h"
#include "Better-Capportal.h"
#include <math.h>
#include <Preferences.h>
#include "captive-portal-web.h"

// Function prototypes
void playStartupMelody();
void playGPSFixBeep();
void playDistanceBeep();
void beep(int frequency, int duration);
void updateBeepInterval(double distance);
void parseLoRaData(String data);
void drawTitleScreen();
void drawConfigScreen();
void drawInitScreen(int step);
void drawRunningScreen();
void updateDisplay();
void updateValueArea(int x, int y, int w, int h, String value, uint16_t color);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2);
void drawArrow(int cx, int cy, int radius, double bearing);
void handleConfigSubmission(std::map<String, String> &data);
void enterConfigMode();
void initializeLoRa();
void loadLoRaConfig();
void saveLoRaConfig();
void initializeSDCard();
int countSDFiles(const char *dirname);
String buildTelemetryString();
void writeToSD();

// Instances
TFT_eSPI tft = TFT_eSPI();
BetterGPS gps;
LoRaRx *loraRx = nullptr;
LoRaConfig loraConfig;
BetterCapportal capportal;
Preferences preferences;

// SD Card uses HSPI (shared with display)
SPIClass sdSPI(HSPI);

// System state
enum AppState
{
  STATE_TITLE,
  STATE_CONFIG,
  STATE_INITIALIZING,
  STATE_RUNNING
};
AppState appState = STATE_TITLE;

// GPS data
struct GPSData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0; // m/s
  int satellites = 0;
  double hdop = 0.0;
  bool hasFix = false;
  double course = 0.0;
} gpsData;

// CanSat data from LoRa
struct CanSatData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  bool valid = false;
  unsigned long lastReceived = 0;
} canSatData;

// Previous values for selective update
struct PreviousValues
{
  double csLat = -999;
  double csLon = -999;
  double csAlt = -999;
  double csSpeed = -999;
  double gpLat = -999;
  double gpLon = -999;
  double gpAlt = -999;
  double gpSpeed = -999;
  int satellites = -1;
  bool gpsFix = false;
  bool csValid = false;
  double distance = -1;
  double bearing = -999;
  double course = -999;
} prevVals;

// Display variables
unsigned long lastDisplayUpdate = 0;
unsigned long titleStartTime = 0;
const unsigned long TITLE_DURATION = 3000; // 3 seconds
const unsigned long LORA_TIMEOUT = 5000;   // 5 seconds

// SD Card
bool sdCardAvailable = false;
unsigned long lastSDWrite = 0;
const unsigned long SD_WRITE_INTERVAL = 1000; // 1 second
String currentLogFile = "";

// Initialization variables
unsigned long initStartTime = 0;
int initStep = 0;
const unsigned long INIT_STEP_DURATION = 1000; // 1 second

// Config mode variables
int lastClientCount = -1;

// Buzzer variables
unsigned long lastBeepTime = 0;
unsigned long beepInterval = 0;
bool hasPlayedGPSFixBeep = false;

// Distance thresholds for beeping
const double MAX_DISTANCE = 3000.0;            // 5000 meters
const double MIN_DISTANCE = 30.0;              // 30 meters
const unsigned long MAX_BEEP_INTERVAL = 10000; // 10 seconds
const unsigned long MIN_BEEP_INTERVAL = 500;   // 0.5 seconds

// Function prototypes
void setup()
{
  // Initialize buzzer
  pinMode(BUZZER, OUTPUT);

  // Retrieve LoRa config
  preferences.begin("cansat", false);
  loadLoRaConfig();

  // Initialize display
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  // Initialize SD Card
  initializeSDCard();

  // Draw intro screen
  titleStartTime = millis();
  drawTitleScreen();

  // Play startup melody
  playStartupMelody();
}

void loop()
{
  unsigned long currentTime = millis();

  // State machine
  switch (appState)
  {
  case STATE_TITLE:
    if (currentTime - titleStartTime >= TITLE_DURATION)
    {
      appState = STATE_CONFIG;
      enterConfigMode();
    }
    break;

  case STATE_CONFIG:
  {
    capportal.handle();

    // Update display if client count changed
    int currentClients = capportal.clientCount();
    if (currentClients != lastClientCount)
    {
      // Client count area
      tft.fillRect(10, 230, 460, 30, TFT_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(10, 230);
      tft.print("Connected clients: ");
      tft.print(currentClients);
      lastClientCount = currentClients;
    }
    break;
  }

  case STATE_INITIALIZING:
    // Initialization sequence
    if (currentTime - initStartTime >= INIT_STEP_DURATION)
    {
      initStep++;
      initStartTime = currentTime;

      switch (initStep)
      {
      case 1:
        drawInitScreen(1);
        break;
      case 2:
        drawInitScreen(2);
        gps.begin(GPS_RX, GPS_TX, 9600);
        break;
      case 3:
        drawInitScreen(3);
        initializeLoRa();
        break;
      case 4:
        drawInitScreen(4);
        break;
      case 5:
        appState = STATE_RUNNING;
        drawRunningScreen();
        hasPlayedGPSFixBeep = false;
        break;
      }
    }
    break;

  case STATE_RUNNING:
    // Update GPS data
    gps.update();

    // Listen for LoRa data
    if (loraRx != nullptr)
    {
      String loraData = loraRx->listen();
      if (loraData.length() > 0)
      {
        // Parse the received data
        parseLoRaData(loraData);
      }
    }

    // Check if LoRa data has timed out
    if (canSatData.valid && (millis() - canSatData.lastReceived > LORA_TIMEOUT))
    {
      canSatData.valid = false;
    }

    // Update GPS data structure
    if (gps.hasFix())
    {
      // Check if this is a new GPS fix
      if (!gpsData.hasFix && !hasPlayedGPSFixBeep)
      {
        playGPSFixBeep();
        hasPlayedGPSFixBeep = true;
      }

      gpsData.hasFix = true;
      gpsData.latitude = gps.getLatitude();
      gpsData.longitude = gps.getLongitude();
      gpsData.altitude = gps.getAltitude();
      gpsData.speed = gps.getSpeedKmph() / 3.6; // Convert km/h to m/s
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
      if (gps.hasCourse())
      {
        gpsData.course = gps.getCourse();
      }
    }
    else
    {
      gpsData.hasFix = false;
      gpsData.latitude = 0.0;
      gpsData.longitude = 0.0;
      gpsData.altitude = 0.0;
      gpsData.speed = 0.0;
      gpsData.satellites = gps.getSatellites();
      gpsData.hdop = gps.getHdop();
      hasPlayedGPSFixBeep = false; // Reset when GPS fix is lost
    }

    // Handle distance-based beeping
    if (gpsData.hasFix && canSatData.valid)
    {
      double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
      updateBeepInterval(distance);

      if (beepInterval > 0 && (currentTime - lastBeepTime >= beepInterval))
      {
        playDistanceBeep();
        lastBeepTime = currentTime;
      }
    }
    else
    {
      beepInterval = 0; // Stop beeping if no valid data
    }

    // Update display with selective redraw
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)
    {
      updateDisplay();
      lastDisplayUpdate = currentTime;
    }

    if (sdCardAvailable && (currentTime - lastSDWrite >= SD_WRITE_INTERVAL))
    {
      writeToSD();
      lastSDWrite = currentTime;
    }
    break;
  }

  delay(10);
}

// ====== BUZZER FUNCTIONS ======

void playStartupMelody()
{
  // Notes: C5, E5, G5, C6
  int melody[] = {523, 659, 784, 1047};
  int durations[] = {150, 150, 150, 300};

  for (int i = 0; i < 4; i++)
  {
    beep(melody[i], durations[i]);
    delay(50);
  }
}

void playGPSFixBeep()
{
  beep(1000, 100);
  delay(100);
  beep(1000, 100);
}

void playDistanceBeep()
{
  // Single short beep for distance indication
  beep(800, 80);
}

void beep(int frequency, int duration)
{
  tone(BUZZER, frequency, duration);
  delay(duration);
  noTone(BUZZER);
}

void updateBeepInterval(double distance)
{
  if (distance >= MAX_DISTANCE)
  {
    beepInterval = MAX_BEEP_INTERVAL;
  }
  else if (distance <= MIN_DISTANCE)
  {
    beepInterval = MIN_BEEP_INTERVAL;
  }
  else
  {
    double ratio = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
    beepInterval = MAX_BEEP_INTERVAL - (ratio * (MAX_BEEP_INTERVAL - MIN_BEEP_INTERVAL));
  }
}

// Parse LoRa data string
void parseLoRaData(String data)
{
  data.trim();

  if (data.length() == 0)
  {
    return;
  }

  // Split by semicolons and store in array
  String values[4] = {"", "", "", ""};
  int valueIndex = 0;
  int startPos = 0;

  for (int i = 0; i <= data.length(); i++)
  {
    if (i == data.length() || data.charAt(i) == ';')
    {
      if (valueIndex < 4)
      {
        values[valueIndex] = data.substring(startPos, i);
        valueIndex++;
      }
      startPos = i + 1;
    }
  }

  // Assign values based on what are received
  canSatData.latitude = values[0].toDouble();
  canSatData.longitude = values[1].toDouble();
  canSatData.altitude = (valueIndex > 2) ? values[2].toDouble() : 0.0;
  canSatData.speed = (valueIndex > 3) ? values[3].toDouble() : 0.0;
  canSatData.valid = true;
  canSatData.lastReceived = millis();
}

// Draw title screen
void drawTitleScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Draw title - larger and centered
  tft.setTextSize(5);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  // Center "CanSat"
  int cansatWidth = 6 * 30;
  tft.setCursor((480 - cansatWidth) / 2, 60);
  tft.println("CanSat");

  // Center "Finder"
  int finderWidth = 6 * 30;
  tft.setCursor((480 - finderWidth) / 2, 120);
  tft.println("Finder");

  // Center "S.E.A.T."
  int seatWidth = 8 * 18;
  tft.setTextSize(3);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor((480 - seatWidth) / 2, 190);
  tft.println("S.E.A.T.");

  // Center "Starting..."
  int startWidth = 12 * 12;
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor((480 - startWidth) / 2, 250);
  tft.println("Starting...");
}

// Draw config screen
void drawConfigScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Title
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(120, 10);
  tft.println("CONFIG MODE");

  // WiFi info
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 60);
  tft.println("WiFi AP: " + String(SSID));
  tft.setCursor(10, 85);
  tft.println("IP: 4.3.2.1");

  // Current settings
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 115);
  tft.println("Current LoRa Settings:");

  // Frequency
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 140);
  tft.print("Freq: ");
  tft.print(loraConfig.frequency);
  tft.println(" Hz");

  // Bandwidth
  tft.setCursor(10, 165);
  tft.print("BW: ");
  tft.print(loraConfig.bandwidth);
  tft.print(" kHz  Sync: ");
  tft.println(loraConfig.sync);

  // Baudrate
  tft.setCursor(10, 190);
  tft.print("Baudrate: ");
  tft.print(loraConfig.baudrate);
  tft.println(" bps");

  // Client count
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 230);
  tft.print("Connected clients: ");
  tft.print(capportal.clientCount());
  lastClientCount = capportal.clientCount();
}

// Draw initialization screen
void drawInitScreen(int step)
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(130, 60);
  tft.println("Initializing...");
  tft.setTextSize(2);

  int yPos = 110;

  // SD Card initialization
  if (step == 1)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, yPos);
    tft.println("SD Card...");
    return;
  }

  // SD Card initialization result
  else if (step >= 2)
  {
    tft.setCursor(100, yPos);
    tft.setTextColor(sdCardAvailable ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.print("SD [");
    tft.print(sdCardAvailable ? "OK" : "FAIL");
    tft.println("]");
    yPos += 30;
  }

  // GPS initialization
  else if (step == 2)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, yPos);
    tft.println("GPS module...");
    return;
  }

  // GPS initialization result
  if (step >= 3)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, yPos);
    tft.println("GPS [OK]");
    yPos += 30;
  }

  // LoRa initialization
  else if (step == 3)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, yPos);
    tft.println("LoRa...");
    return;
  }

  // LoRa initialization result
  if (step >= 4)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(100, yPos);
    tft.println("LoRa [OK]");
    yPos += 30;
  }
}

// Draw running screen - initial setup only
void drawRunningScreen()
{
  tft.fillScreen(TFT_BLACK);

  // Header
  tft.setTextSize(3);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  // CanSat Finder
  int headerWidth = 13 * 18;
  tft.setCursor((480 - headerWidth) / 2, 5);
  tft.println("CanSat Finder");
  tft.drawLine(0, 35, 480, 35, TFT_WHITE);

  // Static labels
  int yPos = 45;
  tft.setTextSize(2);

  // CanSat section
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(5, yPos);
  tft.println("CanSat Data:");
  yPos += 25;
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, yPos);
  tft.println("Lat:");
  tft.setCursor(10, yPos + 20);
  tft.println("Lon:");
  tft.setCursor(10, yPos + 40);
  tft.println("Alt:");
  tft.setCursor(10, yPos + 60);
  tft.println("Speed:");

  // Finder section
  yPos += 90;
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(5, yPos);
  tft.println("Local Data:");
  yPos += 25;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, yPos);
  tft.println("Lat:");
  tft.setCursor(10, yPos + 20);
  tft.println("Lon:");
  tft.setCursor(10, yPos + 40);
  tft.println("Alt:");
  tft.setCursor(10, yPos + 60);
  tft.println("Speed:");
  tft.setCursor(10, yPos + 80);
  tft.println("Sat:");

  // Draw arrow circle in center-right area
  tft.drawCircle(380, 160, 70, TFT_WHITE);

  // Distance label below arrow
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(320, 245);
  tft.println("Distance:");

  // SD Card availability
  tft.setTextSize(1);
  if (sdCardAvailable)
  {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 305);
    tft.println("SD: OK");
  }
  else
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setCursor(10, 305);
    tft.println("SD: FAIL");
  }

  // Reset previous values
  prevVals.csLat = -999;
}

// Update display - selective updates only
void updateDisplay()
{
  int yPos = 70;
  uint16_t csColor = canSatData.valid ? TFT_GREEN : TFT_RED;

  // Update CanSat values if changed
  if (prevVals.csLat != canSatData.latitude)
  {
    updateValueArea(90, yPos, 160, 18, String(canSatData.latitude, 6), csColor);
    prevVals.csLat = canSatData.latitude;
  }
  if (prevVals.csLon != canSatData.longitude)
  {
    updateValueArea(90, yPos + 20, 160, 18, String(canSatData.longitude, 6), csColor);
    prevVals.csLon = canSatData.longitude;
  }
  if (prevVals.csAlt != canSatData.altitude)
  {
    updateValueArea(90, yPos + 40, 160, 18, String(canSatData.altitude, 1) + " m", csColor);
    prevVals.csAlt = canSatData.altitude;
  }
  if (prevVals.csSpeed != canSatData.speed)
  {
    updateValueArea(90, yPos + 60, 160, 18, String(canSatData.speed, 1) + " m/s", csColor);
    prevVals.csSpeed = canSatData.speed;
  }

  // Update validity status if changed
  if (prevVals.csValid != canSatData.valid)
  {
    prevVals.csValid = canSatData.valid;
  }

  yPos = 185;
  uint16_t gpsColor = gpsData.hasFix ? TFT_GREEN : TFT_RED;

  // Update GPS values if changed
  if (prevVals.gpLat != gpsData.latitude)
  {
    updateValueArea(90, yPos, 160, 18, String(gpsData.latitude, 6), gpsColor);
    prevVals.gpLat = gpsData.latitude;
  }
  if (prevVals.gpLon != gpsData.longitude)
  {
    updateValueArea(90, yPos + 20, 160, 18, String(gpsData.longitude, 6), gpsColor);
    prevVals.gpLon = gpsData.longitude;
  }
  if (prevVals.gpAlt != gpsData.altitude)
  {
    updateValueArea(90, yPos + 40, 160, 18, String(gpsData.altitude, 1) + " m", gpsColor);
    prevVals.gpAlt = gpsData.altitude;
  }
  if (prevVals.gpSpeed != gpsData.speed)
  {
    updateValueArea(90, yPos + 60, 160, 18, String(gpsData.speed, 1) + " m/s", gpsColor);
    prevVals.gpSpeed = gpsData.speed;
  }

  // Satellites
  if (prevVals.satellites != gpsData.satellites)
  {
    updateValueArea(90, yPos + 80, 80, 18, String(gpsData.satellites), TFT_WHITE);
    prevVals.satellites = gpsData.satellites;
  }

  // Distance and arrow
  if (gpsData.hasFix && canSatData.valid)
  {
    double distance = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    double absoluteBearing = calculateBearing(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);

    // Update distance display if changed
    if (abs(prevVals.distance - distance) > 0.5)
    {
      String distStr;
      if (distance < 1000)
      {
        distStr = String((int)distance) + " m";
      }
      else
      {
        distStr = String(distance / 1000.0, 2) + " km";
      }
      updateValueArea(300, 265, 150, 18, distStr, TFT_YELLOW);
      prevVals.distance = distance;
    }

    // Determine which bearing to use for the arrow
    double bearingToDisplay = absoluteBearing;
    bool needsRedraw = false;

    // If moving AND have valid course data, use relative bearing
    if (gpsData.speed > 1.0 && gps.hasCourse())
    {
      // Calculate relative bearing (where CanSat is relative to our heading)
      double relativeBearing = absoluteBearing - gpsData.course;

      // Normalize to 0-360 range
      while (relativeBearing < 0)
        relativeBearing += 360;
      while (relativeBearing >= 360)
        relativeBearing -= 360;

      bearingToDisplay = relativeBearing;

      // Redraw if either absolute bearing or course changed significantly
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 1.0 || abs(prevVals.course - gpsData.course) > 2.0);
    }
    else
    {
      // Stationary or no course data available
      // Use absolute bearing (compass direction)
      bearingToDisplay = absoluteBearing;
      needsRedraw = (abs(prevVals.bearing - absoluteBearing) > 1.0);
    }

    if (needsRedraw)
    {
      // Clear arrow area
      tft.fillCircle(380, 160, 69, TFT_BLACK);
      tft.drawCircle(380, 160, 70, TFT_WHITE);

      // Draw arrow
      drawArrow(380, 160, 60, bearingToDisplay);

      // Draw indicator text based on mode
      tft.setTextSize(1);
      if (gpsData.speed > 1.0 && gps.hasCourse())
      {
        // Moving - show "REL" for relative bearing
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(355, 95);
        tft.println("REL");
      }
      else
      {
        // Stationary - show "ABS" for absolute/compass bearing
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(355, 95);
        tft.println("ABS");
      }

      prevVals.bearing = absoluteBearing;
      prevVals.course = gpsData.course;
    }
  }
  else
  {
    // No valid GPS or CanSat data
    if (prevVals.distance != -1)
    {
      updateValueArea(300, 265, 150, 18, "---", TFT_YELLOW);
      prevVals.distance = -1;
      // Clear arrow
      tft.fillCircle(380, 160, 69, TFT_BLACK);
      tft.drawCircle(380, 160, 70, TFT_WHITE);

      // Clear mode indicator
      tft.fillRect(355, 95, 30, 10, TFT_BLACK);

      prevVals.bearing = -999;
      prevVals.course = -999;
    }
  }
}

// Helper to update value area
void updateValueArea(int x, int y, int w, int h, String value, uint16_t color)
{
  tft.fillRect(x, y, w, h, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(color, TFT_BLACK);
  tft.setCursor(x, y);
  tft.println(value);
}

// Calculate bearing from point 1 to point 2
double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(radians(lat2));
  double x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  double bearing = atan2(y, x);
  bearing = degrees(bearing);
  bearing = fmod((bearing + 360.0), 360.0);
  return bearing;
}

double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2)
{
  // Earth's radius in meters
  const double R = 6371000.0;

  // Convert degrees to radians
  double phi1 = radians(lat1);
  double phi2 = radians(lat2);
  double deltaPhi = radians(lat2 - lat1);
  double deltaLambda = radians(lon2 - lon1);

  // Haversine formula
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// Draw arrow pointing to bearing
void drawArrow(int cx, int cy, int radius, double bearing)
{
  // Convert bearing to radians
  double angle = radians(bearing - 90); // -90 to make 0° point up

  // Arrow tip
  int tipX = cx + radius * cos(angle);
  int tipY = cy + radius * sin(angle);

  // Arrow base (opposite side)
  int baseX = cx - radius * 0.3 * cos(angle);
  int baseY = cy - radius * 0.3 * sin(angle);

  // Arrow wings
  double wingAngle1 = angle + radians(150);
  double wingAngle2 = angle - radians(150);

  int wing1X = cx + radius * 0.6 * cos(wingAngle1);
  int wing1Y = cy + radius * 0.6 * sin(wingAngle1);

  int wing2X = cx + radius * 0.6 * cos(wingAngle2);
  int wing2Y = cy + radius * 0.6 * sin(wingAngle2);

  // Draw arrow
  tft.fillTriangle(tipX, tipY, wing1X, wing1Y, baseX, baseY, TFT_GREEN);
  tft.fillTriangle(tipX, tipY, wing2X, wing2Y, baseX, baseY, TFT_GREEN);

  // Draw center dot
  tft.fillCircle(cx, cy, 4, TFT_RED);
}

// Handle configuration submission
void handleConfigSubmission(std::map<String, String> &data)
{
  String newFreq = data["frequency"];
  String newBW = data["bandwidth"];
  String newSync = data["sync"];
  String newBaudStr = data["baudrate"];

  if (newFreq.length() > 0)
  {
    loraConfig.frequency = newFreq;
  }

  if (newBW.length() > 0)
  {
    loraConfig.bandwidth = newBW;
  }

  if (newSync.length() > 0)
  {
    loraConfig.sync = newSync;
  }

  long newBaud = newBaudStr.toInt();
  if (newBaud > 0)
  {
    loraConfig.baudrate = newBaud;
  }
  saveLoRaConfig();

  capportal.stop();
  delay(100);

  appState = STATE_INITIALIZING;
  initStartTime = millis();
  initStep = 0;
}

// Enter configuration mode
void enterConfigMode()
{
  String configHTML = generateConfigHTML(loraConfig);
  capportal.setHTML(configHTML);
  capportal.onSubmit(handleConfigSubmission);
  capportal.begin(SSID, PASSWRD, AP_TIMEOUT);
  drawConfigScreen();
}

// Initialize LoRa
void initializeLoRa()
{
  if (loraRx != nullptr)
  {
    delete loraRx;
    loraRx = nullptr;
  }

  loraRx = new LoRaRx(LORA_RX, LORA_TX, loraConfig.baudrate);

  // Use the configured parameters
  loraRx->begin(loraConfig.frequency, loraConfig.bandwidth, loraConfig.sync);
}

// Load LoRa configuration
void loadLoRaConfig()
{
  loraConfig.frequency = preferences.getString("frequency", "865375000");
  loraConfig.bandwidth = preferences.getString("bandwidth", "250");
  loraConfig.sync = preferences.getString("sync", "12");
  loraConfig.baudrate = preferences.getLong("baudrate", 115200);
}

// Save LoRa configuration
void saveLoRaConfig()
{
  preferences.putString("frequency", loraConfig.frequency);
  preferences.putString("bandwidth", loraConfig.bandwidth);
  preferences.putString("sync", loraConfig.sync);
  preferences.putLong("baudrate", loraConfig.baudrate);
}

void initializeSDCard()
{
  if (!SD_ENABLED)
  {
    sdCardAvailable = false;
    return;
  }

  // Initialize HSPI for SD card
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Give SPI bus time to stabilize
  delay(100);

  // Try to initialize SD card
  if (!SD.begin(SD_CS_PIN, sdSPI))
  {
    sdCardAvailable = false;
    return;
  }

  // Verify SD card is accessible
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    sdCardAvailable = false;
    SD.end();
    return;
  }

  sdCardAvailable = true;

  // Create logs directory if it doesn't exist
  if (!SD.exists("/logs"))
  {
    SD.mkdir("/logs");
  }

  // Create new log file with timestamp
  int fileCount = countSDFiles("/logs");
  currentLogFile = "/logs/telemetry_" + String(fileCount + 1) + ".csv";

  File file = SD.open(currentLogFile, FILE_WRITE);
  if (file)
  {
    String header = "Time,CS_Lat,CS_Lon,CS_Alt,CS_Speed,CS_Valid,GP_Lat,GP_Lon,GP_Alt,GP_Speed,GP_Sats,GP_Fix,Distance,Bearing";
    file.println(header);
    file.close();
  }
  else
  {
    sdCardAvailable = false;
  }
}

// Helper function to count files in directory
int countSDFiles(const char *dirname)
{
  File dir = SD.open(dirname);
  if (!dir || !dir.isDirectory())
  {
    return 0;
  }

  int count = 0;
  File file = dir.openNextFile();
  while (file)
  {
    if (!file.isDirectory())
    {
      count++;
    }
    file.close();
    file = dir.openNextFile();
  }
  dir.close();
  return count;
}

String buildTelemetryString()
{
  String data = String(millis()) + ",";
  data += String(canSatData.latitude, 6) + ",";
  data += String(canSatData.longitude, 6) + ",";
  data += String(canSatData.altitude, 2) + ",";
  data += String(canSatData.speed, 2) + ",";
  data += (canSatData.valid ? "1" : "0") + String(",");
  data += String(gpsData.latitude, 6) + ",";
  data += String(gpsData.longitude, 6) + ",";
  data += String(gpsData.altitude, 2) + ",";
  data += String(gpsData.speed, 2) + ",";
  data += String(gpsData.satellites) + ",";
  data += (gpsData.hasFix ? "1" : "0") + String(",");

  if (gpsData.hasFix && canSatData.valid)
  {
    double dist = calculateDistanceMeters(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    double bear = calculateBearing(gpsData.latitude, gpsData.longitude, canSatData.latitude, canSatData.longitude);
    data += String(dist, 2) + "," + String(bear, 2);
  }
  else
  {
    data += "0.00,0.00";
  }

  return data;
}

void writeToSD()
{
  if (!sdCardAvailable || currentLogFile.length() == 0)
    return;

  String data = buildTelemetryString();

  // Append to current log file
  File file = SD.open(currentLogFile, FILE_APPEND);
  if (file)
  {
    file.println(data);
    file.close();
  }
  else
  {
    // Try to reinitialize on next write
    sdCardAvailable = false;
  }
}