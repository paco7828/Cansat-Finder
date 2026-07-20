#pragma once

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Arduino.h>

class BetterGPS
{
private:
  HardwareSerial gpsSerial;
  TinyGPSPlus gps;
  double courseEmaX = 1.0, courseEmaY = 0.0;
  bool courseEmaInit = false;
  static constexpr double COURSE_EMA_ALPHA = 0.2;

public:
  BetterGPS() : gpsSerial(2) {}

  void begin(uint8_t gpsRx, uint8_t gpsTx = -1, int gpsBaud = 9600)
  {
    gpsSerial.begin(gpsBaud, SERIAL_8N1, gpsRx, gpsTx);

    Serial.println("GPS initialized on Serial2:");
    Serial.println("  RX pin: GPIO " + String(gpsRx) + " (GPS TX → ESP32)");
    if (gpsTx != -1)
    {
      Serial.println("  TX pin: GPIO " + String(gpsTx) + " (GPS RX ← ESP32)");
    }
    Serial.println("  Baud rate: " + String(gpsBaud));
  }

  void update()
  {
    while (gpsSerial.available())
    {
      if (gps.encode(gpsSerial.read()) && gps.course.isValid())
      {
        double courseRad = radians(gps.course.deg());
        double cx = cos(courseRad), cy = sin(courseRad);
        if (!courseEmaInit)
        {
          courseEmaX = cx;
          courseEmaY = cy;
          courseEmaInit = true;
        }
        else
        {
          courseEmaX += (cx - courseEmaX) * COURSE_EMA_ALPHA;
          courseEmaY += (cy - courseEmaY) * COURSE_EMA_ALPHA;
        }
      }
    }
  }

  bool hasFix()
  {
    return gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  }

  double getLatitude()
  {
    return gps.location.lat();
  }

  double getLongitude()
  {
    return gps.location.lng();
  }

  double getSpeedKmph()
  {
    return gps.speed.kmph();
  }

  double getCourse()
  {
    return fmod(degrees(atan2(courseEmaY, courseEmaX)) + 360.0, 360.0);
  }

  bool hasCourse()
  {
    return gps.course.isValid();
  }

  int getSatellites()
  {
    return gps.satellites.value();
  }

  double getHdop()
  {
    return gps.hdop.hdop();
  }
};