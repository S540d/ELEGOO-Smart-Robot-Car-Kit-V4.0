# ESP32 Brain - Installation Guide

## Hardware: AI Thinker ESP32-WROVER-CAM

## Quick Start

### 1. Install ESP32 Board Support in Arduino IDE

1. Open Arduino IDE
2. File → Preferences
3. Add to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Tools → Board → Boards Manager
5. Search "esp32" and install "esp32 by Espressif Systems"

### 2. Configure Board Settings

**Important Settings:**
- Board: "AI Thinker ESP32-CAM"
- Upload Speed: 115200
- Flash Frequency: 80MHz
- Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"

### 3. Wiring for Upload

ESP32-CAM needs an FTDI/USB-Serial adapter for uploading:

```
ESP32-CAM ──→ FTDI Adapter
   5V     ──→    5V
   GND    ──→    GND
   U0R    ──→    TX
   U0T    ──→    RX
   IO0    ──→    GND (only during upload!)
```

**Upload Process:**
1. Connect IO0 to GND
2. Power on ESP32
3. Click Upload in Arduino IDE
4. Wait for "Connecting..."
5. After upload completes, disconnect IO0 from GND
6. Press RESET button

### 4. Connect to Arduino

After uploading, reconnect ESP32 to Arduino:

```
ESP32-CAM ──→ Arduino Uno
   TX (IO1) ──→    RX (D0)
   RX (IO3) ──→    TX (D1)
   GND      ──→    GND
```

**Power:** ESP32 can be powered from Arduino 5V or separate power supply.

### 5. Access Web Interface

1. Power on robot
2. Connect to WiFi:
   - SSID: `ELEGOO-Robot`
   - Password: `elegoo123`
3. Open browser (iOS Safari works great!):
   - http://192.168.4.1
   - or http://elegoo-robot.local

## Features

### Current (Phase 2)
- ✅ WiFi Access Point
- ✅ Web-based control interface
- ✅ Touch-friendly controls (iOS compatible!)
- ✅ Real-time sensor display
- ✅ REST API
- ✅ Arduino ↔ ESP32 communication

### Coming Soon (Phase 3)
- 📹 Live camera stream
- 🎯 Computer vision features
- 🧠 Autonomous navigation
- 📱 Progressive Web App

## API Reference

### POST /api/move
Move the robot
```bash
curl -X POST http://192.168.4.1/api/move \
  -d "direction=forward&speed=150"
```

Directions: `forward`, `backward`, `left`, `right`, `stop`

### GET /api/sensors
Get current sensor readings
```bash
curl http://192.168.4.1/api/sensors
```

Returns JSON:
```json
{
  "ultrasonic": 25,
  "lineL": 512,
  "lineM": 510,
  "lineR": 508,
  "voltage": 7.2,
  "mode": 1
}
```

### POST /api/mode
Change operating mode
```bash
curl -X POST http://192.168.4.1/api/mode -d "mode=1"
```

Modes:
- 0: Standby
- 1: Manual
- 2: Line Follow
- 3: Obstacle Avoidance
- 4: Object Follow
- 5: Autonomous

### POST /api/stop
Emergency stop
```bash
curl -X POST http://192.168.4.1/api/stop
```

## Troubleshooting

### Upload Fails
- Ensure IO0 is connected to GND during upload
- Try pressing RESET button when Arduino IDE shows "Connecting..."
- Check FTDI adapter is 3.3V or 5V compatible

### Can't Connect to WiFi
- Wait 10-15 seconds after power-on
- Check ESP32 LED is blinking (indicates AP active)
- Forget network on phone and reconnect

### Robot Doesn't Move
- Check Serial Monitor (115200 baud) for errors
- Verify Arduino is powered and running
- Check TX/RX connections (TX→RX, RX→TX)
- Ensure Arduino has the updated firmware with ESP32 support

### Web Interface Doesn't Load
- Clear browser cache
- Try IP address instead of hostname
- Check you're connected to "ELEGOO-Robot" WiFi

## Development

### Serial Debugging

ESP32 outputs debug info on Serial (115200 baud):
```
=================================
ELEGOO Robot - ESP32 Brain v1.0
=================================
[WiFi] Setting up Access Point...
[WiFi] AP SSID: ELEGOO-Robot
[WiFi] AP IP: 192.168.4.1
[mDNS] Responder started
[WebServer] HTTP server started
✓ ESP32 Brain Ready!
```

### Modify WiFi Credentials

Edit `esp32_main.ino`:
```cpp
const char* AP_SSID = "YourSSID";
const char* AP_PASSWORD = "YourPassword";
```

## File Structure

```
ESP32-Brain/
├── esp32_main.ino          # Main ESP32 code
├── lib/
│   └── RobotProtocol.h     # Shared protocol (Arduino + ESP32)
└── README.md               # This file
```

## Pin Usage

| ESP32 Pin | Function |
|-----------|----------|
| GPIO 1 (TX) | Serial to Arduino |
| GPIO 3 (RX) | Serial from Arduino |
| GPIO 4 | Camera (don't use) |
| GPIO 0 | Flash LED |
| GPIO 33 | Red LED (on-board) |

## License

ESP32 Brain Architecture: MIT License
Original ELEGOO Code: ELEGOO Inc.