# Mine Safety Monitoring System

An ESP32-based safety monitoring system for underground mines. Two wireless sensor nodes stream temperature, humidity, CO, methane and roof/wall displacement to ThingSpeak; a main controller reads both channels, shows live status on an OLED display and raises alarms with a buzzer and red/green status LED.

## Hardware

**Sensor node (x2, same sketch, change `NODE_ID`):**
- ESP32 Dev Module
- DHT11 - temperature & humidity
- MQ-9 - carbon monoxide + methane
- VL53L0X - laser distance, roof/wall displacement
- XL-B524SURSYGW 5 mm red/green LED (common anode)

**Main controller:**
- ESP32 Dev Module
- SSD1306 0.96" OLED (I2C)
- GY-GPS6MV2 (NEO-6M) GPS
- Buzzer
- XL-B524SURSYGW red/green status LED

## How it works

- Each node publishes to its own ThingSpeak channel (8 fields: temp, humidity, CO, CH4, distance, displacement, status, sensor fault).
- The controller polls both channels every 20 s, checks thresholds and node freshness, and classifies the state:
  - Green, silent - all normal
  - Amber, beep every 2 s - warning / node offline / sensor fault
  - Blinking red, fast beep - DANGER

## Setup

1. Create two ThingSpeak channels (one per node) with fields 1-8 as listed in `sensor_node.ino`.
2. In `sensor_node.ino`: set `WIFI_SSID`, `WIFI_PASS`, `NODE_ID` (1 or 2) and the node's ThingSpeak write API key. Flash to both nodes.
3. In `main_controller.ino`: set WiFi credentials, both channel IDs and read API keys. Flash to the controller.

## Arduino libraries (Library Manager)

- DHT sensor library by Adafruit (+ Adafruit Unified Sensor)
- Adafruit_VL53L0X
- Adafruit SSD1306 + Adafruit GFX Library
- TinyGPSPlus by Mikal Hart
- ArduinoJson by Benoit Blanchon (v7)

Board: ESP32 Dev Module (esp32 by Espressif)
