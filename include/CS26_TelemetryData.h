#pragma once
// ============================================================================
//  CS26_TelemetryData.h – SEAT_CS26_Telemetry lib v1.0.0
// ============================================================================
//  Közös adatmodell: TelemetryData
//    - CanSat oldalon: a szenzor driverek (SystemSensors, GPS, CAN) töltik fel
//    - GND oldalon   : a CS26Telem::parse*() függvények töltik fel
//
//  A struct platform-független (pure data), nincs HW függőség.
//  calculateAltitude() – pure math (hypsometric formula)
//  isStale()           – millis() alapú, Arduino targeten működik
// ============================================================================

#include <Arduino.h>

struct TelemetryData
{
  // ── PCB elérhetőség (CAN PING/PONG alapján) ──────────────────────────────
  bool pdu_alive      = false;
  bool top_alive      = false;
  bool bot_alive      = false;

  // ── PDU PCB – Powerline állapotok ────────────────────────────────────────
  bool pdu_pow_top    = false;  // TOP PCB tápellátás
  bool pdu_pow_bottom = false;  // BOTTOM PCB tápellátás
  bool pdu_pow_camera = false;  // Kamera tápellátás

  // ── MAIN PCB – BME680 ────────────────────────────────────────────────────
  uint32_t ts_bme680     = 0;
  float bme680_temp      = NAN;  // °C
  float bme680_hum       = NAN;  // %
  float bme680_press     = NAN;  // hPa
  float bme680_gas_kohm  = NAN;  // kOhm (gas resistance)
  float bme680_alt_m     = NAN;  // m  (barometrikus)

  // ── MAIN PCB – BNO086 IMU ────────────────────────────────────────────────
  uint32_t ts_imu        = 0;
  float imu_roll         = NAN;  // fok
  float imu_pitch        = NAN;  // fok
  float imu_yaw          = NAN;  // fok
  float imu_linAcc       = NAN;  // m/s² (magnitude)
  float imu_gyroX        = NAN;  // rad/s
  float imu_gyroY        = NAN;  // rad/s
  float imu_gyroZ        = NAN;  // rad/s
  bool  imu_calibrated   = false;
  bool  imu_moving       = false;

  // ── MAIN PCB – GPS ───────────────────────────────────────────────────────
  uint32_t ts_gps        = 0;
  bool     gps_valid     = false;
  float    gps_lat       = NAN;    // fok
  float    gps_lon       = NAN;    // fok
  float    gps_alt_m     = NAN;    // m  (GPS MSL)
  float    gps_speed     = NAN;    // km/h
  float    gps_course    = NAN;    // fok (0 = Észak)
  uint8_t  gps_sats      = 0;
  uint32_t gps_age_ms    = 99999;  // ms az utolsó fix óta
  uint8_t  gps_hour      = 0;
  uint8_t  gps_min       = 0;
  uint8_t  gps_sec       = 0;

  // ── PDU PCB – BME280 (via CAN) ───────────────────────────────────────────
  uint32_t ts_bme280     = 0;
  float bme280_temp      = NAN;  // °C
  float bme280_hum       = NAN;  // %
  float bme280_press     = NAN;  // hPa
  float bme280_alt_m     = NAN;  // m  (barometrikus)

  // ── PDU PCB – INA3221 (via CAN) ──────────────────────────────────────────
  uint32_t ts_ina        = 0;
  float ina_ch1_v        = NAN;  // V   (Battery Vin)
  float ina_ch1_a        = NAN;  // mA
  float ina_ch2_v        = NAN;  // V   (5V buck)
  float ina_ch2_a        = NAN;  // mA
  float ina_ch3_v        = NAN;  // V   (3.3V buck)
  float ina_ch3_a        = NAN;  // mA

  // ── TOP PCB – VEML7700 (via CAN) ─────────────────────────────────────────
  uint32_t ts_veml       = 0;
  float    veml_lux      = NAN;  // lux

  // ── RX metadata – GND oldal tölti, CanSat nem használja ─────────────────
  uint16_t last_rx_seq   = 0;   // utoljára fogadott sorszám
  uint32_t last_rx_ts    = 0;   // CanSat-oldali millis() timestamp
  uint8_t  last_rx_type  = 0;   // utoljára fogadott csomag típusa (1/2/3/10)

  // ── Helper ───────────────────────────────────────────────────────────────
  /// Visszaadja, hogy a megadott timestamp régebbi-e maxAgeMs ms-nál
  bool isStale(uint32_t timestamp, uint32_t maxAgeMs) const
  {
    if (timestamp == 0) return true;
    return (millis() - timestamp) > maxAgeMs;
  }
};

// Pure math – nincs HW függőség (hypsometric formula)
inline float calculateAltitude(float pressure_hpa, float ref_hpa)
{
  if (ref_hpa <= 0.0f || isnan(pressure_hpa)) return NAN;
  return 44330.0f * (1.0f - pow(pressure_hpa / ref_hpa, 0.1903f));
}
