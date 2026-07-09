# Laivelis Pinouts

This file documents the active pin mappings used by the current firmware.

## Remote / Ground Station ESP32

Source: `remote_control/remote_control.ino`

### nRF24L01+

- CE: GPIO 4
- CSN: GPIO 5
- SCK: GPIO 18
- MISO: GPIO 19
- MOSI: GPIO 23

### Joysticks

- Joy 1 X: GPIO 34
- Joy 1 Y: GPIO 35
- Joy 2 X: GPIO 32
- Joy 2 Y: GPIO 33

### Battery Monitor

- Battery ADC: GPIO 36

### OLED SSD1306

- SDA: GPIO 21
- SCL: GPIO 22

### Buzzer

- Buzzer: GPIO 27

### Matrix Keypad

- Rows: GPIO 13, 14, 15, 25, 26
- Columns: GPIO 12, 16

### Wireless / Services

- Bluetooth Classic name: `Boat_Ground_Station`
- AP SSID: `LaivelisRemote`
- AP password: `Laivelis123`

## Boat Receiver ESP32-S3

Source: `boat_receiver/boat_receiver.ino`

### Motor Driver

- IN1: GPIO 15
- IN2: GPIO 16
- IN3: GPIO 17
- IN4: GPIO 18

### Servo / Hopper

- Servo: GPIO 8

### Lights

- Front light: GPIO 1
- Back left light: GPIO 2
- Back right light: GPIO 3

### Temperature Sensor

- DS18B20: GPIO 4

### Battery Monitor

- Battery ADC: GPIO 7

### Battery Status LEDs

- Red: GPIO 35
- Green 1: GPIO 36
- Green 2: GPIO 37
- Green 3: GPIO 38

### GPS

- GPS RX: GPIO 44
- GPS TX: GPIO 43

### IMU (I2C)

- SDA: GPIO 5
- SCL: GPIO 6

### nRF24L01+

- CE: GPIO 9
- CSN: GPIO 10
- SCK: GPIO 12
- MISO: GPIO 13
- MOSI: GPIO 11