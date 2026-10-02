# Motorcycle Navigation Display

ESP32-C3 firmware for a compact motorcycle navigation display using a
128x160 ST7735 TFT. The display receives navigation frames over Bluetooth Low
Energy (BLE) and renders route, maneuver, distance, and connection status.

## Hardware

- ESP32-C3 development board
- 1.8-inch 128x160 ST7735 TFT display

### Wiring

| ESP32-C3 | ST7735 |
| --- | --- |
| GPIO 4 | SCK |
| GPIO 6 | SDA/MOSI |
| GPIO 7 | CS |
| GPIO 2 | A0/DC |
| GPIO 3 | RESET |
| GND | GND |
| 3.3 V | VCC/LED |

Check the markings and voltage requirements of the exact display module before
connecting power.

## Requirements

- Arduino CLI or Arduino IDE
- ESP32 Arduino core
- Adafruit GFX Library 1.12.6
- Adafruit ST7735 and ST7789 Library 1.11.0
- NimBLE-Arduino 2.5.1

The Arduino library manager installs the required Adafruit dependencies.

## Setup

Clone the repository into a directory named `display2`, matching the primary
Arduino sketch filename:

```sh
git clone git@github.com:Kamal0078k/moto-nav-adruino-sketch.git display2
cd display2
```

Install the ESP32 core and libraries with Arduino CLI:

```sh
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install \
  "Adafruit GFX Library@1.12.6" \
  "Adafruit ST7735 and ST7789 Library@1.11.0" \
  "NimBLE-Arduino@2.5.1"
```

## Build

```sh
arduino-cli compile --fqbn esp32:esp32:esp32c3 .
```

## Upload

Replace the example serial port with the port connected to the ESP32-C3:

```sh
arduino-cli upload \
  --fqbn esp32:esp32:esp32c3 \
  --port /dev/cu.usbmodem2101 \
  .
```

## BLE mode

The firmware advertises as `MotoNav Display` and starts in BLE receiver mode by
default. Set `START_IN_BLE_MODE` to `0` before compiling to start in demo mode.

The service exposes separate characteristics for navigation frames and display
capabilities. The protocol implementation is in `display_protocol.cpp` and
`display_protocol.h`.
