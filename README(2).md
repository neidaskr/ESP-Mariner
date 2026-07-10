# ESP-Mariner

ESP-Mariner is a dual-ESP autonomous bait boat system:
- Remote Bridge ESP32: hosts offline WebUI over AP and bridges control/telemetry over nRF24L01+
- Boat Receiver ESP32-S3: executes drive commands, publishes sensor telemetry, and handles failsafe logic

## Repository Layout

- `remote_bridge/` - AP + SD + WebSocket + nRF bridge firmware
- `boat_receiver/` - boat control/sensor/telemetry firmware
- `web_ui/` - static browser UI assets
- `shared/` - protocol docs and shared contracts
- `PINOUTS.md` - pin compatibility map
- `what-to-do.md` - full system requirements

## Current Status

Implemented:
- Browser manual control pipeline (WebSocket -> remote -> nRF -> boat)
- Browser mission upload/commands transport (WebSocket -> remote -> nRF -> boat queue parser)
- Boat-side waypoint mission executor with heading PD steering and waypoint progression
- RTH mode navigation to captured home coordinate
- Runtime geofence enforcement with mission stop/reject behavior
- WebUI geofence configuration panel and transport command
- Mission progress telemetry (running flag, active waypoint, waypoint count)
- Docking-mode behavior for RTH (On Shore decel-to-1m stop, In Boat gentle tap + impact stop)
- CRC16-CCITT packet integrity on command and telemetry packets
- Telemetry diagnostics: dock impact events, CRC invalid RX counter, radio TX fail counter
- Runtime docking tuning from WebUI (dock PWM, impact threshold, RTH halt/decel distances)
- Persistent config on boat via NVS (geofence + tuning restored after reboot)
- Differential motor command mixing and output
- 500 ms failsafe watchdog motor neutral on boat
- nRF telemetry packet types: TEL_CORE, TEL_GPS, TEL_IMU, TEL_POWER
- Remote-side telemetry decoding and WebSocket telemetry publishing
- Boat-side battery + DS18B20 telemetry
- Boat-side live GPS parsing (TinyGPS++) and IMU streaming (MPU6050_light)

Planned next:
- ESP-NOW power/OTA channel (future)

## Build Setup

This repository includes [platformio.ini](platformio.ini) with pinned platforms/libraries.

Examples:
- `pio run -e remote_bridge`
- `pio run -e boat_receiver`

## Arduino Libraries

Install these libraries in Arduino IDE / PlatformIO:
- RF24
- ESPAsyncWebServer
- AsyncTCP
- ArduinoJson
- TinyGPSPlus
- MPU6050_light
- OneWire
- DallasTemperature

## Notes

- Remote bridge serves static WebUI from SD root.
- Keep AP credentials and RF channel aligned on both devices.
- Pin maps currently follow `PINOUTS.md` compatibility mapping.
