# Laivelio V3 Protocol (Draft v0.1)

This document defines the first implementation contract between:
- Browser WebUI <-> Remote Bridge ESP32 (WebSocket JSON)
- Remote Bridge ESP32 <-> Boat Receiver ESP32-S3 (nRF24L01+ binary)

## 1. Timing

- Browser telemetry push: 20 Hz
- Browser control frame send: 20 Hz
- Remote <-> Boat command refresh: 20 Hz
- Boat failsafe watchdog timeout: 500 ms

## 2. WebSocket JSON Messages

All messages include:
- `type`: string
- `ts`: unix ms timestamp (client or bridge clock)

### 2.1 Browser -> Remote

`control.manual`
```json
{
  "type": "control.manual",
  "ts": 1731000000000,
  "x": -1.0,
  "y": 0.4,
  "headingLock": false,
  "targetHeadingDeg": 0,
  "lights": {"front": true, "backLeft": false, "backRight": false},
  "servo": {"drop": false}
}
```

`mission.upload`
```json
{
  "type": "mission.upload",
  "ts": 1731000000000,
  "missionId": 7,
  "landingMode": "ON_SHORE",
  "waypoints": [
    {"lat": 54.123456, "lng": 24.654321},
    {"lat": 54.123556, "lng": 24.654421}
  ]
}
```

`mission.command`
```json
{
  "type": "mission.command",
  "ts": 1731000000000,
  "cmd": "START"
}
```

`config.geofence`
```json
{
  "type": "config.geofence",
  "ts": 1731000000000,
  "enabled": true,
  "centerLat": 54.123456,
  "centerLng": 24.654321,
  "radiusM": 120.0
}
```

`config.tuning`
```json
{
  "type": "config.tuning",
  "ts": 1731000000000,
  "dockPwmPct": 5.0,
  "impactThresholdG": 0.22,
  "rthHaltM": 1.0,
  "rthDecelStartM": 5.0
}
```

### 2.2 Remote -> Browser

`telemetry.frame`
```json
{
  "type": "telemetry.frame",
  "ts": 1731000000000,
  "link": {"rssi": -65, "packetLossPct": 2.4, "lastAckMs": 45},
  "boat": {
    "gps": {"lat": 54.123456, "lng": 24.654321, "speedMps": 0.8, "fix": true},
    "imu": {"headingDeg": 243.5, "yawRateDps": -1.2},
    "battery": {"voltage": 11.5, "percent": 64},
    "tempC": 18.2,
    "mode": "MANUAL",
    "failsafe": false,
    "geofenceOk": true
  }
}
```

## 3. nRF24 Binary Packet

Fixed-size packet: 32 bytes.

```text
Byte 0   : SYNC = 0xA5
Byte 1   : Version = 0x01
Byte 2   : MsgType
Byte 3   : Seq
Byte 4   : Flags
Bytes 5-28: Payload (24 bytes)
Bytes 29-30: CRC16-CCITT (big endian)
Byte 31  : End marker = 0x5A
```

CRC16 is now enforced on both bridge and boat for command and telemetry packets.

### 3.1 MsgType

- `0x10` CMD_MANUAL
- `0x11` CMD_MISSION_META
- `0x12` CMD_MISSION_WAYPOINT
- `0x13` CMD_MISSION_CTRL
- `0x14` CMD_GEOFENCE_CFG
- `0x15` CMD_TUNING_CFG
- `0x20` TEL_CORE
- `0x21` TEL_GPS
- `0x22` TEL_IMU
- `0x23` TEL_POWER
- `0x30` ACK

### 3.2 Command payload examples

`CMD_MANUAL`
- int16 leftMotor (-255..255)
- int16 rightMotor (-255..255)
- uint8 lightMask bit0/1/2
- uint8 servoDrop (0/1)
- uint8 mode

`CMD_MISSION_META`
- uint8 missionId
- uint8 landingMode (ON_SHORE=1, IN_BOAT=2)
- uint8 waypointCount

`CMD_MISSION_WAYPOINT`
- uint8 missionId
- uint8 waypointIndex
- uint8 waypointCount
- int32 latE7
- int32 lngE7

`CMD_MISSION_CTRL`
- uint8 cmd (START=1, PAUSE=2, ABORT=3, RTH=4)
- uint8 landingMode (ON_SHORE=1, IN_BOAT=2)

Landing mode behavior for RTH:
- `ON_SHORE`: proportional deceleration with final halt at 1.0m
- `IN_BOAT`: gentle 5% docking thrust with IMU impact-stop trigger

`CMD_GEOFENCE_CFG`
- uint8 enabled
- int32 centerLatE7
- int32 centerLngE7
- uint16 radiusDeciMeters

`CMD_TUNING_CFG`
- uint8 dockPwm
- uint16 impactThresholdCentiG
- uint16 rthHaltCm
- uint16 rthDecelStartCm

Bridge behavior note:
- `CMD_GEOFENCE_CFG` and `CMD_TUNING_CFG` are persisted by boat firmware in NVS and restored on boot.

### 3.3 Telemetry payload examples

`TEL_CORE`
- uint8 mode
- uint8 failsafe
- uint8 geofenceOk
- uint8 batteryPct
- int16 tempCx10
- uint8 missionRunning
- uint8 activeWaypoint
- uint8 waypointCount
- uint8 dockImpact
- uint16 dockImpactCount
- uint16 crcInvalidRxCount
- uint16 radioTxFailCount

`TEL_GPS`
- int32 latE7
- int32 lngE7
- uint16 speedCms
- uint8 fix

`TEL_IMU`
- int16 headingDegX10
- int16 yawRateDpsX10

`TEL_POWER`
- uint16 batteryVoltageCenti (e.g., 1185 = 11.85V)
- uint8 batteryPct

## 4. Safety rules

- Boat must set motors neutral when no valid command frame in 500 ms.
- Manual command always overrides autonomous mission command.
- Geofence violation in mission mode must stop thrust and emit alert status.

## 5. Versioning

The bridge and boat must refuse packets with unknown protocol `Version`.
