#pragma once

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "constants.h"

class BetterGPS {
private:
  HardwareSerial gpsSerial;
  TinyGPSPlus gps;
  const int GPS_CACHE_VALIDITY_MS = 1000;

  // Cache for Hungarian time to avoid repeated calculations
  struct HungarianTimeCache {
    int year;
    int month;
    int day;
    int dayIndex;
    int hour;
    int minute;
    int second;
    bool valid;
    unsigned long lastUpdate;
  } timeCache;

  // Zeller's Congruence to calculate day of the week
  int calculateDayOfWeek(int y, int m, int d) {
    if (m < 3) {
      m += 12;
      y -= 1;
    }
    int k = y % 100;
    int j = y / 100;
    int f = d + 13 * (m + 1) / 5 + k + k / 4 + j / 4 + 5 * j;

    // Correct Zeller's result: 0=Saturday, 1=Sunday, 2=Monday, etc.
    // Convert to standard: 0=Sunday, 1=Monday, 2=Tuesday, ..., 6=Saturday
    return (f + 1) % 7;
  }

  // Check if it's daylight saving time in Hungary (CEST)
  // DST starts last Sunday in March at 2:00 AM
  // DST ends last Sunday in October at 3:00 AM
  bool isDaylightSavingTime(int year, int month, int day, int hour) {
    // Before March or after October - definitely standard time
    if (month < 3 || month > 10) {
      return false;
    }

    // April to September - definitely daylight saving time
    if (month > 3 && month < 10) {
      return true;
    }

    // March - need to check if we're past the last Sunday
    if (month == 3) {
      // Find the last Sunday of March
      int lastSunday = 31;
      while (calculateDayOfWeek(year, 3, lastSunday) != 0) {
        lastSunday--;
      }

      if (day < lastSunday) {
        return false;  // Before DST starts
      } else if (day > lastSunday) {
        return true;  // After DST starts
      } else {
        // On the last Sunday - DST starts at 2:00 AM UTC (3:00 AM CET)
        return hour >= 1;  // 1:00 UTC = 2:00 CET when DST starts
      }
    }

    // October - need to check if we're before the last Sunday
    if (month == 10) {
      // Find the last Sunday of October
      int lastSunday = 31;
      while (calculateDayOfWeek(year, 10, lastSunday) != 0) {
        lastSunday--;
      }

      if (day < lastSunday) {
        return true;  // Before DST ends
      } else if (day > lastSunday) {
        return false;  // After DST ends
      } else {
        // On the last Sunday - DST ends at 3:00 AM CEST (1:00 AM UTC)
        return hour < 1;  // Before 1:00 UTC (2:00 AM CET when DST ends)
      }
    }

    return false;
  }

  // Get Hungarian timezone offset (+1 CET, +2 CEST)
  int getHungarianTimezoneOffset(int year, int month, int day, int hour) {
    return isDaylightSavingTime(year, month, day, hour) ? 2 : 1;
  }

  // Convert UTC time to Hungarian local time
  void convertToHungarianTime(int &year, int &month, int &day, int &hour, int &minute, int &second) {
    // First, get the timezone offset based on UTC time
    int offset = getHungarianTimezoneOffset(year, month, day, hour);

    // Add the offset to the hour
    hour += offset;

    // Handle day overflow
    if (hour >= 24) {
      hour -= 24;
      day++;

      // Days in each month
      int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

      // Check for leap year
      bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
      if (isLeapYear) daysInMonth[1] = 29;

      // Handle month overflow
      if (day > daysInMonth[month - 1]) {
        day = 1;
        month++;

        // Handle year overflow
        if (month > 12) {
          month = 1;
          year++;
        }
      }
    }

    // Handle day underflow
    else if (hour < 0) {
      hour += 24;
      day--;

      // Handle day underflow
      if (day < 1) {
        month--;

        // Handle month underflow
        if (month < 1) {
          month = 12;
          year--;
        }

        // Days in each month
        int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

        // Check for leap year
        bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        if (isLeapYear) daysInMonth[1] = 29;

        day = daysInMonth[month - 1];
      }
    }
  }

  // Update the cache with fresh Hungarian time calculation
  void updateTimeCache() {
    if (!hasFix()) {
      timeCache.valid = false;
      return;
    }

    // Get UTC time from GPS
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Convert to Hungarian time
    convertToHungarianTime(year, month, day, hour, minute, second);

    // Update cache
    timeCache.year = year;
    timeCache.month = month;
    timeCache.day = day;
    timeCache.hour = hour;
    timeCache.minute = minute;
    timeCache.second = second;
    timeCache.dayIndex = calculateDayOfWeek(year, month, day);
    timeCache.valid = true;
    timeCache.lastUpdate = millis();
  }

  // Check if cache is still valid
  bool isCacheValid() {
    return timeCache.valid && (millis() - timeCache.lastUpdate < GPS_CACHE_VALIDITY_MS);
  }

public:
  BetterGPS()
    : gpsSerial(1) {
    timeCache.valid = false;
    timeCache.lastUpdate = 0;
  }

  void begin(byte gpsRx, byte gpsTx = -1, int gpsBaud = 9600) {
    gpsSerial.begin(gpsBaud, SERIAL_8N1, gpsRx, gpsTx);
  }

  void update() {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        // New data available, invalidate cache
        timeCache.valid = false;
      }
    }
  }

  bool hasFix() {
    return gps.location.isValid() && gps.date.isValid() && gps.time.isValid();
  }

  double getLatitude() {
    return gps.location.lat();
  }

  double getLongitude() {
    return gps.location.lng();
  }

  double getSpeedKmph() {
    return gps.speed.kmph();
  }

  // Function that calculates Hungarian time once and fills all values
  void getHungarianTime(int &year, int &month, int &day, int &dayIndex, int &hour, int &minute, int &second) {
    if (!hasFix()) {
      year = month = day = dayIndex = hour = minute = second = 0;
      return;
    }

    // Check if we need to update the cache
    if (!isCacheValid()) {
      updateTimeCache();
    }

    // Return cached values
    if (timeCache.valid) {
      year = timeCache.year;
      month = timeCache.month;
      day = timeCache.day;
      dayIndex = timeCache.dayIndex;
      hour = timeCache.hour;
      minute = timeCache.minute;
      second = timeCache.second;
    } else {
      year = month = day = dayIndex = hour = minute = second = 0;
    }
  }

  // Optimized getter functions - use cache instead of recalculating
  int getYear() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.year : 0;
  }

  byte getMonth() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.month : 0;
  }

  byte getDay() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.day : 0;
  }

  byte getHour() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.hour : 0;
  }

  byte getMinute() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.minute : 0;
  }

  byte getSecond() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.second : 0;
  }

  byte getDayIndex() {
    if (!isCacheValid()) {
      updateTimeCache();
    }
    return timeCache.valid ? timeCache.dayIndex : 0;
  }
};