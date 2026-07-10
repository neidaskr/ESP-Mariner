#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include <ArduinoJson.h>
#include <math.h>

// AP settings from V3 spec
static const char *AP_SSID = "Laivelio_Bridge";
static const char *AP_PASS = "Laivelio123";

// Pin compatibility from PINOUTS.md (legacy map kept as requested)
static const int NRF_CE_PIN = 4;
static const int NRF_CSN_PIN = 5;
static const int NRF_SCK_PIN = 18;
static const int NRF_MISO_PIN = 19;
static const int NRF_MOSI_PIN = 23;

// SD card CS pin is configurable; set according to your module wiring.
static const int SD_CS_PIN = 14;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

uint8_t g_seq = 0;
uint32_t g_lastTelemetryPushMs = 0;

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

struct TelemetryState {
  bool validCore = false;
  bool validGps = false;
  bool validImu = false;
  bool validPower = false;

  uint8_t mode = 0;
  bool failsafe = false;
  bool geofenceOk = true;
  uint8_t batteryPct = 0;
  float tempC = 0.0f;
  bool missionRunning = false;
  uint8_t activeWaypoint = 0;
  uint8_t waypointCount = 0;
  bool dockImpact = false;
  uint16_t dockImpactCount = 0;
  uint16_t crcInvalidRxCount = 0;
  uint16_t radioTxFailCount = 0;

  double lat = 0.0;
  double lng = 0.0;
  float speedMps = 0.0f;
  bool gpsFix = false;

  float headingDeg = 0.0f;
  float yawRateDps = 0.0f;
  float batteryVoltage = 0.0f;

  uint32_t totalRx = 0;
  uint32_t invalidRx = 0;
  uint32_t lastBoatPacketMs = 0;
} g_telemetry;

struct ManualControl {
  float x = 0.0f;
  float y = 0.0f;
  bool headingLock = false;
  int targetHeadingDeg = 0;
  bool lightFront = false;
  bool lightBackLeft = false;
  bool lightBackRight = false;
  bool servoDrop = false;
} g_control;

static int16_t readI16Be(const uint8_t *buf, size_t idx) {
  return static_cast<int16_t>((static_cast<uint16_t>(buf[idx]) << 8) | buf[idx + 1]);
}

static int32_t readI32Be(const uint8_t *buf, size_t idx) {
  return static_cast<int32_t>((static_cast<uint32_t>(buf[idx]) << 24) |
                              (static_cast<uint32_t>(buf[idx + 1]) << 16) |
                              (static_cast<uint32_t>(buf[idx + 2]) << 8) |
                              static_cast<uint32_t>(buf[idx + 3]));
}

static uint16_t readU16Be(const uint8_t *buf, size_t idx) {
  return static_cast<uint16_t>((static_cast<uint16_t>(buf[idx]) << 8) | buf[idx + 1]);
}

static void writeI32Be(uint8_t *buf, size_t idx, int32_t v) {
  buf[idx] = static_cast<uint8_t>((v >> 24) & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  buf[idx + 2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  buf[idx + 3] = static_cast<uint8_t>(v & 0xFF);
}

static void writeU16Be(uint8_t *buf, size_t idx, uint16_t v) {
  buf[idx] = static_cast<uint8_t>((v >> 8) & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>(v & 0xFF);
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

static const char *modeToLabel(uint8_t mode) {
  switch (mode) {
    case 1:
      return "MANUAL";
    case 2:
      return "AUTONOMOUS";
    case 3:
      return "RTH";
    default:
      return "UNKNOWN";
  }
}

static void handleTelemetryPacket(const uint8_t *pkt, size_t n) {
  g_telemetry.totalRx++;
  if (!validatePacket(pkt, n)) {
    g_telemetry.invalidRx++;
    return;
  }

  g_telemetry.lastBoatPacketMs = millis();

  const uint8_t msgType = pkt[2];
  if (msgType == TEL_CORE) {
    g_telemetry.mode = pkt[5];
    g_telemetry.failsafe = pkt[6] != 0;
    g_telemetry.geofenceOk = pkt[7] != 0;
    g_telemetry.batteryPct = pkt[8];
    g_telemetry.tempC = static_cast<float>(readI16Be(pkt, 9)) / 10.0f;
    g_telemetry.missionRunning = pkt[11] != 0;
    g_telemetry.activeWaypoint = pkt[12];
    g_telemetry.waypointCount = pkt[13];
    g_telemetry.dockImpact = pkt[14] != 0;
    g_telemetry.dockImpactCount = readU16Be(pkt, 15);
    g_telemetry.crcInvalidRxCount = readU16Be(pkt, 17);
    g_telemetry.radioTxFailCount = readU16Be(pkt, 19);
    g_telemetry.validCore = true;
    return;
  }

  if (msgType == TEL_GPS) {
    const int32_t latE7 = readI32Be(pkt, 5);
    const int32_t lngE7 = readI32Be(pkt, 9);
    const uint16_t speedCms = readU16Be(pkt, 13);
    g_telemetry.lat = static_cast<double>(latE7) / 10000000.0;
    g_telemetry.lng = static_cast<double>(lngE7) / 10000000.0;
    g_telemetry.speedMps = static_cast<float>(speedCms) / 100.0f;
    g_telemetry.gpsFix = pkt[15] != 0;
    g_telemetry.validGps = true;
    return;
  }

  if (msgType == TEL_IMU) {
    g_telemetry.headingDeg = static_cast<float>(readI16Be(pkt, 5)) / 10.0f;
    g_telemetry.yawRateDps = static_cast<float>(readI16Be(pkt, 7)) / 10.0f;
    g_telemetry.validImu = true;
    return;
  }

  if (msgType == TEL_POWER) {
    const uint16_t cv = readU16Be(pkt, 5);
    g_telemetry.batteryVoltage = static_cast<float>(cv) / 100.0f;
    g_telemetry.validPower = true;
    return;
  }

  g_telemetry.invalidRx++;
}

static void handleRadioRx() {
  while (radio.available()) {
    uint8_t pkt[32] = {0};
    radio.read(&pkt, sizeof(pkt));
    handleTelemetryPacket(pkt, sizeof(pkt));
  }
}

static void sendPacket(uint8_t msgType, const uint8_t *payload, size_t payloadLen, uint8_t flags = 0) {
  uint8_t pkt[32] = {0};
  pkt[0] = PKT_SYNC;
  pkt[1] = PKT_VER;
  pkt[2] = msgType;
  pkt[3] = g_seq++;
  pkt[4] = flags;

  if (payload != nullptr && payloadLen > 0) {
    const size_t capped = payloadLen > 24 ? 24 : payloadLen;
    memcpy(&pkt[5], payload, capped);
  }

  finalizePacket(pkt);

  radio.stopListening();
  radio.write(&pkt, sizeof(pkt));
  radio.startListening();
}

// Converts virtual joystick axes into differential motor targets.
static void mixMotors(float x, float y, int16_t &leftOut, int16_t &rightOut) {
  float left = y + x;
  float right = y - x;

  if (left > 1.0f) left = 1.0f;
  if (left < -1.0f) left = -1.0f;
  if (right > 1.0f) right = 1.0f;
  if (right < -1.0f) right = -1.0f;

  leftOut = static_cast<int16_t>(left * 255.0f);
  rightOut = static_cast<int16_t>(right * 255.0f);
}

static void sendManualPacket() {
  int16_t left = 0;
  int16_t right = 0;
  mixMotors(g_control.x, g_control.y, left, right);

  uint8_t payload[24] = {0};
  payload[0] = static_cast<uint8_t>((left >> 8) & 0xFF);
  payload[1] = static_cast<uint8_t>(left & 0xFF);
  payload[2] = static_cast<uint8_t>((right >> 8) & 0xFF);
  payload[3] = static_cast<uint8_t>(right & 0xFF);

  uint8_t lightMask = 0;
  if (g_control.lightFront) lightMask |= 0x01;
  if (g_control.lightBackLeft) lightMask |= 0x02;
  if (g_control.lightBackRight) lightMask |= 0x04;
  payload[4] = lightMask;
  payload[5] = g_control.servoDrop ? 1 : 0;
  sendPacket(CMD_MANUAL, payload, sizeof(payload));
}

static uint8_t parseLandingMode(const char *landingMode) {
  if (landingMode != nullptr && strcmp(landingMode, "IN_BOAT") == 0) {
    return 2;
  }
  return 1;
}

static uint8_t parseMissionCmd(const char *cmd) {
  if (cmd == nullptr) return 0;
  if (strcmp(cmd, "START") == 0) return 1;
  if (strcmp(cmd, "PAUSE") == 0) return 2;
  if (strcmp(cmd, "ABORT") == 0) return 3;
  if (strcmp(cmd, "RTH") == 0) return 4;
  return 0;
}

static void sendMissionUpload(JsonDocument &doc) {
  const uint8_t missionId = doc["missionId"] | 1;
  const char *landingMode = doc["landingMode"] | "ON_SHORE";
  const uint8_t landingModeCode = parseLandingMode(landingMode);

  JsonArray waypoints = doc["waypoints"].as<JsonArray>();
  uint8_t totalWp = static_cast<uint8_t>(waypoints.size() > 255 ? 255 : waypoints.size());

  uint8_t payloadMeta[24] = {0};
  payloadMeta[0] = missionId;
  payloadMeta[1] = landingModeCode;
  payloadMeta[2] = totalWp;
  sendPacket(CMD_MISSION_META, payloadMeta, sizeof(payloadMeta));

  uint8_t idx = 0;
  for (JsonObject wp : waypoints) {
    if (idx >= totalWp) break;

    const double lat = wp["lat"] | 0.0;
    const double lng = wp["lng"] | 0.0;
    const int32_t latE7 = static_cast<int32_t>(llround(lat * 10000000.0));
    const int32_t lngE7 = static_cast<int32_t>(llround(lng * 10000000.0));

    uint8_t payloadWp[24] = {0};
    payloadWp[0] = missionId;
    payloadWp[1] = idx;
    payloadWp[2] = totalWp;
    writeI32Be(payloadWp, 4, latE7);
    writeI32Be(payloadWp, 8, lngE7);
    sendPacket(CMD_MISSION_WAYPOINT, payloadWp, sizeof(payloadWp));

    idx++;
  }

  Serial.printf("Mission uploaded id=%u waypoints=%u\n", missionId, totalWp);
}

static void sendMissionCommand(JsonDocument &doc) {
  const char *cmd = doc["cmd"] | "";
  const uint8_t cmdCode = parseMissionCmd(cmd);
  if (cmdCode == 0) {
    Serial.println("Unknown mission.command cmd");
    return;
  }

  const char *landingMode = doc["landingMode"] | "ON_SHORE";
  const uint8_t landingModeCode = parseLandingMode(landingMode);

  uint8_t payloadCtrl[24] = {0};
  payloadCtrl[0] = cmdCode;
  payloadCtrl[1] = landingModeCode;
  sendPacket(CMD_MISSION_CTRL, payloadCtrl, sizeof(payloadCtrl));

  Serial.printf("Mission command sent cmd=%u\n", cmdCode);
}

static void sendGeofenceConfig(JsonDocument &doc) {
  const bool enabled = doc["enabled"] | false;
  const double centerLat = doc["centerLat"] | 0.0;
  const double centerLng = doc["centerLng"] | 0.0;
  const double radiusM = doc["radiusM"] | 0.0;

  uint8_t payload[24] = {0};
  payload[0] = enabled ? 1 : 0;
  writeI32Be(payload, 1, static_cast<int32_t>(llround(centerLat * 10000000.0)));
  writeI32Be(payload, 5, static_cast<int32_t>(llround(centerLng * 10000000.0)));

  double dm = radiusM * 10.0;
  if (dm < 0.0) dm = 0.0;
  if (dm > 65535.0) dm = 65535.0;
  writeU16Be(payload, 9, static_cast<uint16_t>(llround(dm)));

  sendPacket(CMD_GEOFENCE_CFG, payload, sizeof(payload));
  Serial.printf("Geofence cfg sent enabled=%u radius=%.1f\n", enabled ? 1 : 0, radiusM);
}

static void sendTuningConfig(JsonDocument &doc) {
  double dockPwmPct = doc["dockPwmPct"] | 5.0;
  double impactThresholdG = doc["impactThresholdG"] | 0.22;
  double rthHaltM = doc["rthHaltM"] | 1.0;
  double rthDecelStartM = doc["rthDecelStartM"] | 5.0;

  if (dockPwmPct < 1.0) dockPwmPct = 1.0;
  if (dockPwmPct > 40.0) dockPwmPct = 40.0;
  if (impactThresholdG < 0.05) impactThresholdG = 0.05;
  if (impactThresholdG > 3.0) impactThresholdG = 3.0;
  if (rthHaltM < 0.2) rthHaltM = 0.2;
  if (rthHaltM > 8.0) rthHaltM = 8.0;
  if (rthDecelStartM < rthHaltM + 0.2) rthDecelStartM = rthHaltM + 0.2;
  if (rthDecelStartM > 50.0) rthDecelStartM = 50.0;

  const uint16_t impactCentiG = static_cast<uint16_t>(llround(impactThresholdG * 100.0));
  const uint16_t haltCm = static_cast<uint16_t>(llround(rthHaltM * 100.0));
  const uint16_t decelCm = static_cast<uint16_t>(llround(rthDecelStartM * 100.0));
  const uint8_t dockPwm = static_cast<uint8_t>(llround((dockPwmPct / 100.0) * 255.0));

  uint8_t payload[24] = {0};
  payload[0] = dockPwm;
  writeU16Be(payload, 1, impactCentiG);
  writeU16Be(payload, 3, haltCm);
  writeU16Be(payload, 5, decelCm);

  sendPacket(CMD_TUNING_CFG, payload, sizeof(payload));
  Serial.printf("Tuning cfg sent dockPwm=%u impact=%.2f halt=%.2f decel=%.2f\n",
                dockPwm, impactThresholdG, rthHaltM, rthDecelStartM);
}

static void pushTelemetryToWeb() {
  StaticJsonDocument<512> doc;
  doc["type"] = "telemetry.frame";
  doc["ts"] = millis();

  JsonObject link = doc.createNestedObject("link");
  link["rssi"] = -60;
  if (g_telemetry.totalRx > 0) {
    link["packetLossPct"] = (100.0f * static_cast<float>(g_telemetry.invalidRx)) /
                            static_cast<float>(g_telemetry.totalRx);
  } else {
    link["packetLossPct"] = 0.0f;
  }
  link["lastAckMs"] = g_telemetry.lastBoatPacketMs > 0 ? (millis() - g_telemetry.lastBoatPacketMs) : -1;

  JsonObject boat = doc.createNestedObject("boat");
  JsonObject gps = boat.createNestedObject("gps");
  gps["lat"] = g_telemetry.lat;
  gps["lng"] = g_telemetry.lng;
  gps["speedMps"] = g_telemetry.speedMps;
  gps["fix"] = g_telemetry.gpsFix;

  JsonObject imu = boat.createNestedObject("imu");
  imu["headingDeg"] = g_telemetry.headingDeg;
  imu["yawRateDps"] = g_telemetry.yawRateDps;

  JsonObject battery = boat.createNestedObject("battery");
  battery["voltage"] = g_telemetry.batteryVoltage;
  battery["percent"] = g_telemetry.batteryPct;

  boat["tempC"] = g_telemetry.tempC;
  boat["mode"] = modeToLabel(g_telemetry.mode);
  boat["failsafe"] = g_telemetry.failsafe;
  boat["geofenceOk"] = g_telemetry.geofenceOk;
  boat["missionRunning"] = g_telemetry.missionRunning;
  boat["activeWaypoint"] = g_telemetry.activeWaypoint;
  boat["waypointCount"] = g_telemetry.waypointCount;
  boat["dockImpact"] = g_telemetry.dockImpact;
  boat["dockImpactCount"] = g_telemetry.dockImpactCount;
  boat["crcInvalidRxCount"] = g_telemetry.crcInvalidRxCount;
  boat["radioTxFailCount"] = g_telemetry.radioTxFailCount;

  String payload;
  serializeJson(doc, payload);
  ws.textAll(payload);
}

static void handleWsMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = reinterpret_cast<AwsFrameInfo *>(arg);
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    return;
  }

  const char *type = doc["type"] | "";

  if (strcmp(type, "control.manual") == 0) {
    g_control.x = doc["x"] | 0.0f;
    g_control.y = doc["y"] | 0.0f;
    g_control.headingLock = doc["headingLock"] | false;
    g_control.targetHeadingDeg = doc["targetHeadingDeg"] | 0;

    JsonObject lights = doc["lights"];
    g_control.lightFront = lights["front"] | false;
    g_control.lightBackLeft = lights["backLeft"] | false;
    g_control.lightBackRight = lights["backRight"] | false;

    JsonObject servo = doc["servo"];
    g_control.servoDrop = servo["drop"] | false;
  }

  if (strcmp(type, "mission.upload") == 0) {
    sendMissionUpload(doc);
  }

  if (strcmp(type, "mission.command") == 0) {
    sendMissionCommand(doc);
  }

  if (strcmp(type, "config.geofence") == 0) {
    sendGeofenceConfig(doc);
  }

  if (strcmp(type, "config.tuning") == 0) {
    sendTuningConfig(doc);
  }
}

static void onWsEvent(AsyncWebSocket *serverRef, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  (void)serverRef;
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client connected: %u\n", client->id());
    return;
  }
  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client disconnected: %u\n", client->id());
    return;
  }
  if (type == WS_EVT_DATA) {
    handleWsMessage(arg, data, len);
  }
}

static void setupSdAndRoutes() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed, static web files unavailable.");
  } else {
    server.serveStatic("/", SD, "/").setDefaultFile("index.html");
    Serial.println("Serving WebUI from SD card");
  }

  server.on("/api/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"ok\":true,\"service\":\"remote_bridge\"}");
  });
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

  const uint8_t txAddr[6] = "BRG01";
  const uint8_t rxAddr[6] = "BOT01";
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();

  Serial.println("nRF24 ready");
}

void setup() {
  Serial.begin(115200);
  delay(250);

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apOk) {
    Serial.println("Failed to start AP");
  } else {
    Serial.print("AP started. IP: ");
    Serial.println(WiFi.softAPIP());
  }

  setupRadio();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  setupSdAndRoutes();
  server.begin();

  Serial.println("Remote bridge ready");
}

void loop() {
  const uint32_t now = millis();

  handleRadioRx();

  // 20 Hz command refresh to keep the boat watchdog fed.
  static uint32_t lastCmdMs = 0;
  if (now - lastCmdMs >= 50) {
    lastCmdMs = now;
    sendManualPacket();
  }

  // 20 Hz telemetry push to browser clients.
  if (now - g_lastTelemetryPushMs >= 50) {
    g_lastTelemetryPushMs = now;
    pushTelemetryToWeb();
  }

  ws.cleanupClients();
}
