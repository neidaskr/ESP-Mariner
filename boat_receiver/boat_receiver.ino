#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPSPlus.h>
#include <MPU6050_light.h>
#include <math.h>
#include <Preferences.h>

// Pin compatibility from PINOUTS.md
static const int MOTOR_IN1 = 15;
static const int MOTOR_IN2 = 16;
static const int MOTOR_IN3 = 17;
static const int MOTOR_IN4 = 18;

static const int SERVO_PIN = 8;

static const int LIGHT_FRONT_PIN = 1;
static const int LIGHT_BACK_LEFT_PIN = 2;
static const int LIGHT_BACK_RIGHT_PIN = 3;

static const int DS18B20_PIN = 4;
static const int BATTERY_ADC_PIN = 7;

static const int BAT_LED_RED = 35;
static const int BAT_LED_G1 = 36;
static const int BAT_LED_G2 = 37;
static const int BAT_LED_G3 = 38;

static const int GPS_RX_PIN = 44;
static const int GPS_TX_PIN = 43;

static const int I2C_SDA_PIN = 5;
static const int I2C_SCL_PIN = 6;

static const int NRF_CE_PIN = 9;
static const int NRF_CSN_PIN = 10;
static const int NRF_SCK_PIN = 12;
static const int NRF_MISO_PIN = 13;
static const int NRF_MOSI_PIN = 11;

static const uint32_t FAILSAFE_TIMEOUT_MS = 500;

static const uint8_t PKT_SYNC = 0xA5;
static const uint8_t PKT_VER = 0x01;
static const uint8_t PKT_END = 0x5A;

static const uint8_t TEL_CORE = 0x20;
static const uint8_t TEL_GPS = 0x21;
static const uint8_t TEL_IMU = 0x22;
static const uint8_t TEL_POWER = 0x23;

static const uint8_t CMD_MANUAL = 0x10;
static const uint8_t CMD_MISSION_META = 0x11;
static const uint8_t CMD_MISSION_WAYPOINT = 0x12;
static const uint8_t CMD_MISSION_CTRL = 0x13;
static const uint8_t CMD_GEOFENCE_CFG = 0x14;
static const uint8_t CMD_TUNING_CFG = 0x15;

static const uint8_t MODE_MANUAL = 1;
static const uint8_t MODE_AUTONOMOUS = 2;
static const uint8_t MODE_RTH = 3;

static const uint8_t MISSION_CMD_START = 1;
static const uint8_t MISSION_CMD_PAUSE = 2;
static const uint8_t MISSION_CMD_ABORT = 3;
static const uint8_t MISSION_CMD_RTH = 4;
static const uint8_t LANDING_ON_SHORE = 1;
static const uint8_t LANDING_IN_BOAT = 2;

static const uint8_t MISSION_MAX_WAYPOINTS = 64;
static const int16_t MANUAL_OVERRIDE_DEADBAND = 12;
static const float NAV_KP = 3.5f;
static const float NAV_KD = 0.5f;
static const int16_t NAV_CRUISE_PWM = 160;
static const float ARRIVAL_RADIUS_M = 2.0f;
static const float NAV_DECEL_START_M = 5.0f;
static const float RTH_HALT_M = 1.0f;
static const float RTH_DECEL_START_M = 5.0f;
static const int16_t IN_BOAT_DOCK_PWM = 13;
static const float IN_BOAT_DOCK_MAX_DIST_M = 3.0f;
static const float IMPACT_THRESHOLD_G = 0.22f;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
TinyGPSPlus gps;
MPU6050 mpu(Wire);
Preferences prefs;

int16_t g_leftCmd = 0;
int16_t g_rightCmd = 0;
uint8_t g_lightMask = 0;
uint8_t g_servoDrop = 0;
uint32_t g_lastValidCmdMs = 0;
uint8_t g_telSeq = 0;

bool g_imuReady = false;
int32_t g_latE7 = 0;
int32_t g_lngE7 = 0;
uint16_t g_speedCms = 0;
bool g_gpsFix = false;
float g_headingDeg = 0.0f;
float g_yawRateDps = 0.0f;
float g_accelDeltaG = 0.0f;
bool g_lastDockImpact = false;
uint16_t g_dockImpactCount = 0;
uint16_t g_crcInvalidRxCount = 0;
uint16_t g_radioTxFailCount = 0;
float g_rthHaltM = RTH_HALT_M;
float g_rthDecelStartM = RTH_DECEL_START_M;
int16_t g_inBoatDockPwm = IN_BOAT_DOCK_PWM;
float g_impactThresholdG = IMPACT_THRESHOLD_G;

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void savePersistentConfig() {
  prefs.putBool("gf_en", g_geofenceEnabled);
  prefs.putInt("gf_lat", g_geofenceCenterLatE7);
  prefs.putInt("gf_lng", g_geofenceCenterLngE7);
  prefs.putFloat("gf_rad", g_geofenceRadiusM);

  prefs.putShort("dock_pwm", static_cast<int16_t>(g_inBoatDockPwm));
  prefs.putFloat("impact_g", g_impactThresholdG);
  prefs.putFloat("rth_halt", g_rthHaltM);
  prefs.putFloat("rth_decel", g_rthDecelStartM);
}

static void loadPersistentConfig() {
  g_geofenceEnabled = prefs.getBool("gf_en", false);
  g_geofenceCenterLatE7 = prefs.getInt("gf_lat", 0);
  g_geofenceCenterLngE7 = prefs.getInt("gf_lng", 0);
  g_geofenceRadiusM = prefs.getFloat("gf_rad", 0.0f);
  if (g_geofenceRadiusM < 0.5f) {
    g_geofenceEnabled = false;
    g_geofenceRadiusM = 0.0f;
  }

  g_inBoatDockPwm = prefs.getShort("dock_pwm", IN_BOAT_DOCK_PWM);
  if (g_inBoatDockPwm > 255) g_inBoatDockPwm = 255;
  if (g_inBoatDockPwm < -255) g_inBoatDockPwm = -255;
  if (g_inBoatDockPwm < 3) g_inBoatDockPwm = 3;

  g_impactThresholdG = clampf(prefs.getFloat("impact_g", IMPACT_THRESHOLD_G), 0.05f, 3.0f);
  g_rthHaltM = clampf(prefs.getFloat("rth_halt", RTH_HALT_M), 0.2f, 8.0f);
  g_rthDecelStartM = prefs.getFloat("rth_decel", RTH_DECEL_START_M);
  if (g_rthDecelStartM < g_rthHaltM + 0.2f) {
    g_rthDecelStartM = g_rthHaltM + 0.2f;
  }
  if (g_rthDecelStartM > 50.0f) {
    g_rthDecelStartM = 50.0f;
  }
}

struct MissionWaypoint {
  int32_t latE7;
  int32_t lngE7;
};

MissionWaypoint g_waypoints[MISSION_MAX_WAYPOINTS];
uint8_t g_missionId = 0;
uint8_t g_landingMode = 1;
uint8_t g_waypointTarget = 0;
uint8_t g_waypointLoaded = 0;
bool g_missionArmed = false;
bool g_missionRunning = false;
uint8_t g_mode = MODE_MANUAL;
uint8_t g_activeWpIdx = 0;
int32_t g_homeLatE7 = 0;
int32_t g_homeLngE7 = 0;
bool g_homeValid = false;
bool g_geofenceEnabled = false;
int32_t g_geofenceCenterLatE7 = 0;
int32_t g_geofenceCenterLngE7 = 0;
float g_geofenceRadiusM = 0.0f;
bool g_geofenceOk = true;

static int32_t readI32Be(const uint8_t *buf, size_t idx) {
  return static_cast<int32_t>((static_cast<uint32_t>(buf[idx]) << 24) |
                              (static_cast<uint32_t>(buf[idx + 1]) << 16) |
                              (static_cast<uint32_t>(buf[idx + 2]) << 8) |
                              static_cast<uint32_t>(buf[idx + 3]));
}

static uint16_t readU16Be(const uint8_t *buf, size_t idx) {
  return static_cast<uint16_t>((static_cast<uint16_t>(buf[idx]) << 8) | buf[idx + 1]);
}

static uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

static void writeU16Be(uint8_t *buf, size_t idx, uint16_t v) {
  buf[idx] = static_cast<uint8_t>((v >> 8) & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>(v & 0xFF);
}

static void finalizePacket(uint8_t *pkt) {
  const uint16_t crc = crc16Ccitt(pkt, 29);
  pkt[29] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  pkt[30] = static_cast<uint8_t>(crc & 0xFF);
  pkt[31] = PKT_END;
}

static bool validatePacket(const uint8_t *pkt, size_t n) {
  if (n != 32) return false;
  if (pkt[0] != PKT_SYNC || pkt[1] != PKT_VER || pkt[31] != PKT_END) return false;
  const uint16_t expected = static_cast<uint16_t>((static_cast<uint16_t>(pkt[29]) << 8) | pkt[30]);
  const uint16_t actual = crc16Ccitt(pkt, 29);
  return expected == actual;
}

static float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static float e7ToDeg(int32_t e7) {
  return static_cast<float>(e7) / 10000000.0f;
}

static float degToRad(float deg) {
  return deg * (3.14159265359f / 180.0f);
}

static float distanceMeters(float lat1Deg, float lon1Deg, float lat2Deg, float lon2Deg) {
  const float lat1 = degToRad(lat1Deg);
  const float lon1 = degToRad(lon1Deg);
  const float lat2 = degToRad(lat2Deg);
  const float lon2 = degToRad(lon2Deg);
  const float dLat = lat2 - lat1;
  const float dLon = lon2 - lon1;

  const float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
                  cosf(lat1) * cosf(lat2) * sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
  const float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  return 6371000.0f * c;
}

static float bearingDeg(float lat1Deg, float lon1Deg, float lat2Deg, float lon2Deg) {
  const float lat1 = degToRad(lat1Deg);
  const float lon1 = degToRad(lon1Deg);
  const float lat2 = degToRad(lat2Deg);
  const float lon2 = degToRad(lon2Deg);
  const float dLon = lon2 - lon1;

  const float x = sinf(dLon) * cosf(lat2);
  const float y = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
  float brg = atan2f(x, y) * (180.0f / 3.14159265359f);
  if (brg < 0.0f) brg += 360.0f;
  return brg;
}

static int16_t clampPwm(int32_t v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return static_cast<int16_t>(v);
}

static int16_t navBasePwmProfile(float distM, float haltM, float decelStartM) {
  if (distM <= haltM) return 0;
  if (distM >= decelStartM) return NAV_CRUISE_PWM;

  const float t = (distM - haltM) / (decelStartM - haltM);
  const float clamped = t < 0.05f ? 0.05f : t;
  return static_cast<int16_t>(NAV_CRUISE_PWM * clamped);
}

static int16_t navBasePwm(float distM) {
  return navBasePwmProfile(distM, ARRIVAL_RADIUS_M, NAV_DECEL_START_M);
}

static bool detectDockImpact() {
  const bool hit = g_accelDeltaG >= g_impactThresholdG;
  if (hit && !g_lastDockImpact) {
    g_dockImpactCount++;
  }
  g_lastDockImpact = hit;
  return hit;
}

static void stopMission() {
  g_missionRunning = false;
  g_mode = MODE_MANUAL;
  g_leftCmd = 0;
  g_rightCmd = 0;
}

static void runMissionController() {
  if (!g_missionRunning) return;
  if (!g_gpsFix) {
    g_leftCmd = 0;
    g_rightCmd = 0;
    return;
  }

  int32_t targetLatE7 = 0;
  int32_t targetLngE7 = 0;

  if (g_mode == MODE_RTH) {
    if (!g_homeValid) {
      stopMission();
      return;
    }
    targetLatE7 = g_homeLatE7;
    targetLngE7 = g_homeLngE7;
  } else {
    if (!g_missionArmed || g_waypointTarget == 0 || g_activeWpIdx >= g_waypointTarget) {
      stopMission();
      return;
    }
    targetLatE7 = g_waypoints[g_activeWpIdx].latE7;
    targetLngE7 = g_waypoints[g_activeWpIdx].lngE7;
  }

  const float curLat = e7ToDeg(g_latE7);
  const float curLng = e7ToDeg(g_lngE7);
  const float tgtLat = e7ToDeg(targetLatE7);
  const float tgtLng = e7ToDeg(targetLngE7);

  const float distM = distanceMeters(curLat, curLng, tgtLat, tgtLng);
  if (g_mode == MODE_RTH && g_landingMode == LANDING_ON_SHORE && distM <= RTH_HALT_M) {
    stopMission();
    return;
  }

  if (g_mode == MODE_RTH && g_landingMode == LANDING_IN_BOAT) {
    if ((distM <= IN_BOAT_DOCK_MAX_DIST_M && detectDockImpact()) || distM <= 0.4f) {
      stopMission();
      return;
    }
  }

  if (distM <= ARRIVAL_RADIUS_M) {
    if (g_mode == MODE_RTH) {
      stopMission();
      return;
    }

    g_activeWpIdx++;
    if (g_activeWpIdx >= g_waypointTarget) {
      stopMission();
      return;
    }
    return;
  }

  const float targetHeading = bearingDeg(curLat, curLng, tgtLat, tgtLng);
  const float err = wrap180(targetHeading - g_headingDeg);
  const float correction = NAV_KP * err - NAV_KD * g_yawRateDps;
  int16_t base = navBasePwm(distM);

  if (g_mode == MODE_RTH && g_landingMode == LANDING_ON_SHORE) {
    base = navBasePwmProfile(distM, g_rthHaltM, g_rthDecelStartM);
  }

  if (g_mode == MODE_RTH && g_landingMode == LANDING_IN_BOAT) {
    base = g_inBoatDockPwm;
  }

  g_leftCmd = clampPwm(static_cast<int32_t>(lroundf(base + correction)));
  g_rightCmd = clampPwm(static_cast<int32_t>(lroundf(base - correction)));
}

static void enforceGeofence() {
  if (!g_geofenceEnabled || !g_gpsFix) {
    g_geofenceOk = true;
    return;
  }

  const float curLat = e7ToDeg(g_latE7);
  const float curLng = e7ToDeg(g_lngE7);
  const float cenLat = e7ToDeg(g_geofenceCenterLatE7);
  const float cenLng = e7ToDeg(g_geofenceCenterLngE7);
  const float d = distanceMeters(curLat, curLng, cenLat, cenLng);

  g_geofenceOk = d <= g_geofenceRadiusM;
  if (!g_geofenceOk) {
    g_leftCmd = 0;
    g_rightCmd = 0;
    if (g_missionRunning) {
      stopMission();
    }
  }
}

static int pwmFromSigned(int16_t v) {
  int16_t av = abs(v);
  if (av > 255) av = 255;
  return av;
}

static void applyMotorSide(int16_t cmd, int pinFwd, int pinRev) {
  const int pwm = pwmFromSigned(cmd);
  if (cmd > 0) {
    analogWrite(pinFwd, pwm);
    analogWrite(pinRev, 0);
  } else if (cmd < 0) {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, pwm);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, 0);
  }
}

static void applyOutputs() {
  applyMotorSide(g_leftCmd, MOTOR_IN1, MOTOR_IN2);
  applyMotorSide(g_rightCmd, MOTOR_IN3, MOTOR_IN4);

  digitalWrite(LIGHT_FRONT_PIN, (g_lightMask & 0x01) ? HIGH : LOW);
  digitalWrite(LIGHT_BACK_LEFT_PIN, (g_lightMask & 0x02) ? HIGH : LOW);
  digitalWrite(LIGHT_BACK_RIGHT_PIN, (g_lightMask & 0x04) ? HIGH : LOW);

  // TODO: replace with proper servo library and calibrated pulse widths.
  if (g_servoDrop) {
    analogWrite(SERVO_PIN, 40);
  } else {
    analogWrite(SERVO_PIN, 0);
  }
}

static float readBatteryVoltage() {
  const int raw = analogRead(BATTERY_ADC_PIN);
  const float vAdc = (static_cast<float>(raw) / 4095.0f) * 3.3f;

  // Divider from spec: 100k high / 10k low => scale factor 11x
  return vAdc * 11.0f;
}

static uint8_t batteryPercentFromVoltage(float v) {
  // Conservative 3S Li-ion mapping.
  if (v >= 12.4f) return 100;
  if (v >= 12.1f) return 80;
  if (v >= 11.7f) return 60;
  if (v >= 11.4f) return 40;
  if (v >= 11.0f) return 20;
  return 5;
}

static void updateBatteryLeds(uint8_t pct) {
  digitalWrite(BAT_LED_G3, pct >= 75 ? HIGH : LOW);
  digitalWrite(BAT_LED_G2, pct >= 50 ? HIGH : LOW);
  digitalWrite(BAT_LED_G1, pct >= 25 ? HIGH : LOW);
  digitalWrite(BAT_LED_RED, pct < 25 ? HIGH : LOW);
}

static void parseCommandPacket(const uint8_t *pkt, size_t n) {
  if (!validatePacket(pkt, n)) {
    g_crcInvalidRxCount++;
    return;
  }

  const uint8_t msgType = pkt[2];

  if (msgType == CMD_MANUAL) {
    int16_t left = static_cast<int16_t>((pkt[5] << 8) | pkt[6]);
    int16_t right = static_cast<int16_t>((pkt[7] << 8) | pkt[8]);

    if (left > 255) left = 255;
    if (left < -255) left = -255;
    if (right > 255) right = 255;
    if (right < -255) right = -255;

    const bool manualOverride = (abs(left) > MANUAL_OVERRIDE_DEADBAND) ||
                                (abs(right) > MANUAL_OVERRIDE_DEADBAND);

    if (manualOverride) {
      g_leftCmd = left;
      g_rightCmd = right;
      g_mode = MODE_MANUAL;
      g_missionRunning = false;
    }

    g_lightMask = pkt[9];
    g_servoDrop = pkt[10];
    g_lastValidCmdMs = millis();
    return;
  }

  if (msgType == CMD_MISSION_META) {
    const uint8_t missionId = pkt[5];
    const uint8_t landingMode = pkt[6];
    const uint8_t count = pkt[7];
    const uint8_t cappedCount = count > MISSION_MAX_WAYPOINTS ? MISSION_MAX_WAYPOINTS : count;

    g_missionId = missionId;
    g_landingMode = landingMode;
    g_waypointTarget = cappedCount;
    g_waypointLoaded = 0;
    g_activeWpIdx = 0;
    g_missionArmed = false;
    g_missionRunning = false;
    g_mode = MODE_MANUAL;
    g_lastValidCmdMs = millis();
    Serial.printf("Mission meta id=%u wp=%u\n", g_missionId, g_waypointTarget);
    return;
  }

  if (msgType == CMD_MISSION_WAYPOINT) {
    const uint8_t missionId = pkt[5];
    const uint8_t idx = pkt[6];
    const uint8_t total = pkt[7];
    if (missionId != g_missionId) return;

    const uint8_t cappedTotal = total > MISSION_MAX_WAYPOINTS ? MISSION_MAX_WAYPOINTS : total;
    if (idx >= cappedTotal) return;

    g_waypointTarget = cappedTotal;
    g_waypoints[idx].latE7 = readI32Be(pkt, 9);
    g_waypoints[idx].lngE7 = readI32Be(pkt, 13);

    if (idx + 1 > g_waypointLoaded) {
      g_waypointLoaded = idx + 1;
    }

    if (g_waypointLoaded >= g_waypointTarget && g_waypointTarget > 0) {
      g_missionArmed = true;
    }

    g_lastValidCmdMs = millis();
    return;
  }

  if (msgType == CMD_MISSION_CTRL) {
    const uint8_t cmd = pkt[5];
    const uint8_t landingMode = pkt[6];
    g_landingMode = landingMode;

    if (cmd == MISSION_CMD_START) {
      if (g_missionArmed && g_geofenceOk) {
        g_activeWpIdx = 0;
        g_missionRunning = true;
        g_mode = MODE_AUTONOMOUS;
      }
    } else if (cmd == MISSION_CMD_PAUSE) {
      g_missionRunning = false;
      g_mode = MODE_MANUAL;
      g_leftCmd = 0;
      g_rightCmd = 0;
    } else if (cmd == MISSION_CMD_ABORT) {
      g_missionRunning = false;
      g_missionArmed = false;
      g_waypointLoaded = 0;
      g_waypointTarget = 0;
      g_mode = MODE_MANUAL;
      g_leftCmd = 0;
      g_rightCmd = 0;
    } else if (cmd == MISSION_CMD_RTH) {
      g_missionRunning = true;
      g_mode = MODE_RTH;
    }

    g_lastValidCmdMs = millis();
    Serial.printf("Mission cmd=%u mode=%u armed=%u\n", cmd, g_mode, g_missionArmed ? 1 : 0);
    return;
  }

  if (msgType == CMD_GEOFENCE_CFG) {
    g_geofenceEnabled = pkt[5] != 0;
    g_geofenceCenterLatE7 = readI32Be(pkt, 6);
    g_geofenceCenterLngE7 = readI32Be(pkt, 10);
    g_geofenceRadiusM = static_cast<float>(readU16Be(pkt, 14)) / 10.0f;
    if (g_geofenceRadiusM < 0.5f) {
      g_geofenceEnabled = false;
    }
    savePersistentConfig();
    Serial.printf("Geofence cfg enabled=%u radius=%.1f\n", g_geofenceEnabled ? 1 : 0, g_geofenceRadiusM);
    return;
  }

  if (msgType == CMD_TUNING_CFG) {
    const uint8_t dockPwm = pkt[5];
    const uint16_t impactCentiG = readU16Be(pkt, 6);
    const uint16_t haltCm = readU16Be(pkt, 8);
    const uint16_t decelCm = readU16Be(pkt, 10);

    g_inBoatDockPwm = dockPwm < 3 ? 3 : dockPwm;
    g_impactThresholdG = static_cast<float>(impactCentiG) / 100.0f;
    if (g_impactThresholdG < 0.05f) g_impactThresholdG = 0.05f;
    if (g_impactThresholdG > 3.0f) g_impactThresholdG = 3.0f;

    g_rthHaltM = static_cast<float>(haltCm) / 100.0f;
    if (g_rthHaltM < 0.2f) g_rthHaltM = 0.2f;

    g_rthDecelStartM = static_cast<float>(decelCm) / 100.0f;
    if (g_rthDecelStartM < g_rthHaltM + 0.2f) {
      g_rthDecelStartM = g_rthHaltM + 0.2f;
    }

    savePersistentConfig();

    Serial.printf("Tuning cfg dock=%d impact=%.2f halt=%.2f decel=%.2f\n",
                  g_inBoatDockPwm, g_impactThresholdG, g_rthHaltM, g_rthDecelStartM);
    return;
  }
}

static void handleRadioRx() {
  while (radio.available()) {
    uint8_t pkt[32] = {0};
    radio.read(&pkt, sizeof(pkt));
    parseCommandPacket(pkt, sizeof(pkt));
  }
}

static void runFailsafe() {
  const uint32_t now = millis();
  if (now - g_lastValidCmdMs > FAILSAFE_TIMEOUT_MS) {
    g_leftCmd = 0;
    g_rightCmd = 0;
    g_missionRunning = false;
    g_mode = MODE_MANUAL;
  }
}

static float wrapHeadingDeg(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static void updateGpsState() {
  while (Serial2.available() > 0) {
    gps.encode(static_cast<char>(Serial2.read()));
  }

  const bool validFix = gps.location.isValid() && gps.location.age() < 2000;
  g_gpsFix = validFix;
  if (validFix) {
    g_latE7 = static_cast<int32_t>(gps.location.lat() * 10000000.0);
    g_lngE7 = static_cast<int32_t>(gps.location.lng() * 10000000.0);
    if (!g_homeValid) {
      g_homeLatE7 = g_latE7;
      g_homeLngE7 = g_lngE7;
      g_homeValid = true;
    }
  }

  if (gps.speed.isValid()) {
    const float cms = gps.speed.mps() * 100.0f;
    if (cms < 0.0f) {
      g_speedCms = 0;
    } else if (cms > 65535.0f) {
      g_speedCms = 65535;
    } else {
      g_speedCms = static_cast<uint16_t>(cms);
    }
  }
}

static void updateImuState() {
  if (!g_imuReady) return;

  mpu.update();
  g_headingDeg = wrapHeadingDeg(mpu.getAngleZ());
  g_yawRateDps = mpu.getGyroZ();
  const float ax = mpu.getAccX();
  const float ay = mpu.getAccY();
  const float az = mpu.getAccZ();
  const float mag = sqrtf(ax * ax + ay * ay + az * az);
  g_accelDeltaG = fabsf(mag - 1.0f);
}

static void sendTelemetryPacket(uint8_t msgType, const uint8_t *payload, size_t payloadLen) {
  uint8_t pkt[32] = {0};
  pkt[0] = PKT_SYNC;
  pkt[1] = PKT_VER;
  pkt[2] = msgType;
  pkt[3] = g_telSeq++;

  if (payload != nullptr && payloadLen > 0) {
    const size_t capped = payloadLen > 24 ? 24 : payloadLen;
    memcpy(&pkt[5], payload, capped);
  }

  finalizePacket(pkt);

  radio.stopListening();
  if (!radio.write(&pkt, sizeof(pkt))) {
    g_radioTxFailCount++;
  }
  radio.startListening();
}

static void sendTelemetryBurst() {
  static uint8_t which = 0;

  const float vBat = readBatteryVoltage();
  const uint8_t pct = batteryPercentFromVoltage(vBat);
  updateBatteryLeds(pct);

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  if (tempC < -100.0f || tempC > 120.0f) {
    tempC = 0.0f;
  }

  const int16_t tempX10 = static_cast<int16_t>(tempC * 10.0f);
  const bool failsafeNow = (millis() - g_lastValidCmdMs > FAILSAFE_TIMEOUT_MS);

  uint8_t payload[24] = {0};

  if (which == 0) {
    payload[0] = g_mode;
    payload[1] = failsafeNow ? 1 : 0;
    payload[2] = g_geofenceOk ? 1 : 0;
    payload[3] = pct;
    payload[4] = static_cast<uint8_t>((tempX10 >> 8) & 0xFF);
    payload[5] = static_cast<uint8_t>(tempX10 & 0xFF);
    payload[6] = g_missionRunning ? 1 : 0;
    payload[7] = g_activeWpIdx;
    payload[8] = g_waypointTarget;
    payload[9] = g_lastDockImpact ? 1 : 0;
    writeU16Be(payload, 10, g_dockImpactCount);
    writeU16Be(payload, 12, g_crcInvalidRxCount);
    writeU16Be(payload, 14, g_radioTxFailCount);
    sendTelemetryPacket(TEL_CORE, payload, 24);
  } else if (which == 1) {
    const int32_t latE7 = g_latE7;
    const int32_t lngE7 = g_lngE7;
    const uint16_t speedCms = g_speedCms;
    payload[0] = static_cast<uint8_t>((latE7 >> 24) & 0xFF);
    payload[1] = static_cast<uint8_t>((latE7 >> 16) & 0xFF);
    payload[2] = static_cast<uint8_t>((latE7 >> 8) & 0xFF);
    payload[3] = static_cast<uint8_t>(latE7 & 0xFF);
    payload[4] = static_cast<uint8_t>((lngE7 >> 24) & 0xFF);
    payload[5] = static_cast<uint8_t>((lngE7 >> 16) & 0xFF);
    payload[6] = static_cast<uint8_t>((lngE7 >> 8) & 0xFF);
    payload[7] = static_cast<uint8_t>(lngE7 & 0xFF);
    payload[8] = static_cast<uint8_t>((speedCms >> 8) & 0xFF);
    payload[9] = static_cast<uint8_t>(speedCms & 0xFF);
    payload[10] = g_gpsFix ? 1 : 0;
    sendTelemetryPacket(TEL_GPS, payload, 24);
  } else if (which == 2) {
    const int16_t headingX10 = static_cast<int16_t>(g_headingDeg * 10.0f);
    const int16_t yawRateX10 = static_cast<int16_t>(g_yawRateDps * 10.0f);
    payload[0] = static_cast<uint8_t>((headingX10 >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(headingX10 & 0xFF);
    payload[2] = static_cast<uint8_t>((yawRateX10 >> 8) & 0xFF);
    payload[3] = static_cast<uint8_t>(yawRateX10 & 0xFF);
    sendTelemetryPacket(TEL_IMU, payload, 24);
  } else {
    const uint16_t vCenti = static_cast<uint16_t>(vBat * 100.0f);
    payload[0] = static_cast<uint8_t>((vCenti >> 8) & 0xFF);
    payload[1] = static_cast<uint8_t>(vCenti & 0xFF);
    payload[2] = pct;
    sendTelemetryPacket(TEL_POWER, payload, 24);
  }

  which = static_cast<uint8_t>((which + 1) % 4);
}

static void setupPins() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  pinMode(SERVO_PIN, OUTPUT);

  pinMode(LIGHT_FRONT_PIN, OUTPUT);
  pinMode(LIGHT_BACK_LEFT_PIN, OUTPUT);
  pinMode(LIGHT_BACK_RIGHT_PIN, OUTPUT);

  pinMode(BAT_LED_RED, OUTPUT);
  pinMode(BAT_LED_G1, OUTPUT);
  pinMode(BAT_LED_G2, OUTPUT);
  pinMode(BAT_LED_G3, OUTPUT);

  analogReadResolution(12);
}

static void setupRadio() {
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  if (!radio.begin()) {
    Serial.println("nRF24 init failed");
    return;
  }

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(76);

  const uint8_t rxAddr[6] = "BRG01";
  const uint8_t txAddr[6] = "BOT01";
  radio.openReadingPipe(1, rxAddr);
  radio.openWritingPipe(txAddr);
  radio.startListening();

  Serial.println("nRF24 ready");
}

void setup() {
  Serial.begin(115200);
  delay(250);

  setupPins();
  prefs.begin("mariner", false);
  loadPersistentConfig();

  Serial.printf("Loaded config: gf=%u rad=%.1f dock=%d impact=%.2f halt=%.2f decel=%.2f\n",
                g_geofenceEnabled ? 1 : 0,
                g_geofenceRadiusM,
                g_inBoatDockPwm,
                g_impactThresholdG,
                g_rthHaltM,
                g_rthDecelStartM);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  tempSensor.begin();

  byte imuStatus = mpu.begin();
  if (imuStatus == 0) {
    mpu.calcGyroOffsets(false);
    g_imuReady = true;
    Serial.println("MPU6050 ready");
  } else {
    g_imuReady = false;
    Serial.printf("MPU6050 init failed: %u\n", imuStatus);
  }

  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  setupRadio();

  g_lastValidCmdMs = millis();
  applyOutputs();

  Serial.println("Boat receiver ready");
}

void loop() {
  handleRadioRx();
  runFailsafe();
  updateGpsState();
  updateImuState();
  runMissionController();
  enforceGeofence();
  applyOutputs();

  static uint32_t lastTelemetryMs = 0;
  const uint32_t now = millis();
  if (now - lastTelemetryMs >= 100) {
    lastTelemetryMs = now;
    sendTelemetryBurst();
  }
}
