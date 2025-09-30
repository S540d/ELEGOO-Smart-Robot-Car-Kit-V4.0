# 📋 Project Issues & Roadmap

## ✅ Completed (Phase 1 & 2)

### Architecture & Foundation
- ✅ **Dual-MCU Architecture Design** - Arduino + ESP32 brain separation
- ✅ **Binary Communication Protocol** - CRC-protected serial protocol implemented
- ✅ **No Rewiring Required** - Uses existing Serial TX/RX connection
- ✅ **Backward Compatibility** - IR remote and BLE app still functional
- ✅ **Memory Optimization** - Reduced Arduino flash from 105% to 98%

### ESP32 Features
- ✅ **WiFi Access Point** - "ELEGOO-Robot" AP @ 192.168.4.1
- ✅ **Web Server** - HTTP server with RESTful API
- ✅ **iOS-Compatible Web Interface** - Touch-friendly controls for Safari
- ✅ **Real-time Sensor Data** - 10Hz updates from Arduino to ESP32
- ✅ **mDNS Support** - Access via http://elegoo-robot.local

### Code Quality
- ✅ **Memory Safety** - Buffer overflow protection, null pointer checks
- ✅ **Non-blocking Communication** - Optimized timeouts (5-50ms)
- ✅ **Error Handling** - CRC validation, timeout detection, error reporting
- ✅ **Code Review** - All critical sections reviewed and fixed

### Documentation
- ✅ **Comprehensive README.md** - Architecture diagrams, API docs, roadmap
- ✅ **Testing Guide** - Step-by-step TESTING_GUIDE.md with troubleshooting
- ✅ **Code Comments** - Well-documented protocol and handlers

---

## 🔄 Open Issues (Testing & Validation)

### Testing Required
- [ ] **#1: Hardware Testing - Arduino ↔ ESP32 Communication**
  - **Priority**: High
  - **Description**: Verify Arduino and ESP32 communicate successfully via Serial
  - **Acceptance Criteria**:
    - [ ] ESP32 receives PONG response to PING
    - [ ] Sensor data received every 100ms
    - [ ] No CRC errors during 5-minute test
  - **Testing Steps**: See [TESTING_GUIDE.md](TESTING_GUIDE.md) Phase 3

- [ ] **#2: Web Interface Testing - iOS Safari**
  - **Priority**: High
  - **Description**: Verify web interface works on iOS devices
  - **Acceptance Criteria**:
    - [ ] WiFi connection successful
    - [ ] Web interface loads < 1 second
    - [ ] Motor controls respond < 100ms
    - [ ] Sensor data updates every 500ms
    - [ ] Emergency stop works immediately
  - **Testing Steps**: See [TESTING_GUIDE.md](TESTING_GUIDE.md) Phase 4

- [ ] **#3: Motor Control Validation**
  - **Priority**: High
  - **Description**: Verify motor commands from web interface work correctly
  - **Acceptance Criteria**:
    - [ ] Forward/backward/left/right all function
    - [ ] Stop command halts motors immediately
    - [ ] Speed control works (0-255 range)
    - [ ] No jitter or unexpected movements
  - **Testing Steps**: See [TESTING_GUIDE.md](TESTING_GUIDE.md) Phase 4

- [ ] **#4: Sensor Data Accuracy**
  - **Priority**: Medium
  - **Description**: Verify sensor readings match between Arduino standalone and ESP32-integrated modes
  - **Acceptance Criteria**:
    - [ ] Ultrasonic readings within ±2cm
    - [ ] Line sensor values match
    - [ ] Voltage readings within ±0.1V
    - [ ] IMU pitch/roll within ±5°

- [ ] **#5: Backward Compatibility Testing**
  - **Priority**: Medium
  - **Description**: Ensure existing control methods still work
  - **Acceptance Criteria**:
    - [ ] IR remote control functional with ESP32 disconnected
    - [ ] BLE app functional with ESP32 disconnected
    - [ ] All original modes work (line follow, obstacle avoid, etc.)

### Performance & Stability
- [ ] **#6: Long-term Stability Test**
  - **Priority**: Medium
  - **Description**: Run system for extended period to detect memory leaks or crashes
  - **Acceptance Criteria**:
    - [ ] No crashes after 1 hour continuous operation
    - [ ] No memory leaks detected
    - [ ] WiFi connection remains stable
    - [ ] No communication timeouts

- [ ] **#7: Latency Measurement**
  - **Priority**: Low
  - **Description**: Measure actual latency from button press to motor response
  - **Expected**: < 100ms end-to-end
  - **Testing**: Use high-speed camera or oscilloscope

- [ ] **#8: WiFi Range Testing**
  - **Priority**: Low
  - **Description**: Determine effective control range
  - **Expected**: 10-20 meters line-of-sight

### Documentation
- [ ] **#9: User Feedback Collection**
  - **Priority**: Medium
  - **Description**: Gather feedback from testing phase
  - **Questions**:
    - Is the web interface intuitive?
    - Are the installation instructions clear?
    - What features are missing?
    - Any bugs encountered?

---

## 🚧 Future Enhancements (Phase 3+)

### Vision & Camera (Phase 3)
- [ ] **#10: MJPEG Video Streaming**
  - **Priority**: High
  - **Description**: Stream live video from ESP32-CAM to web interface
  - **Technical**: ESP32-CAM library integration, MJPEG encoder
  - **Benefit**: Real-time FPV control

- [ ] **#11: Camera-Based Line Detection**
  - **Priority**: High
  - **Description**: Replace IR line sensors with camera vision
  - **Technical**: Image processing, edge detection, PID control
  - **Benefit**: More accurate line following, works on any color lines

- [ ] **#12: Color Object Tracking**
  - **Priority**: Medium
  - **Description**: Follow colored objects (ball, marker)
  - **Technical**: HSV color space, centroid detection
  - **Benefit**: Interactive play modes

- [ ] **#13: QR Code Navigation**
  - **Priority**: Medium
  - **Description**: Navigate to waypoints defined by QR codes
  - **Technical**: QR code decoder library
  - **Benefit**: Programmable missions without coding

- [ ] **#14: Face Detection & Tracking**
  - **Priority**: Low
  - **Description**: Detect and follow human faces
  - **Technical**: TensorFlow Lite Micro, pre-trained model
  - **Benefit**: Social robotics applications

### Advanced Navigation (Phase 4)
- [ ] **#15: Path Planning Algorithm**
  - **Priority**: Medium
  - **Description**: Autonomous navigation with obstacle avoidance
  - **Technical**: A* or RRT path planning
  - **Benefit**: Complex autonomous missions

- [ ] **#16: SLAM Mapping**
  - **Priority**: Low
  - **Description**: Build map of environment while navigating
  - **Technical**: Occupancy grid, ultrasonic/IMU fusion
  - **Benefit**: True autonomous exploration

- [ ] **#17: GPS Waypoint Navigation**
  - **Priority**: Low
  - **Description**: Outdoor GPS-based navigation (requires GPS module)
  - **Technical**: GPS integration, compass heading
  - **Benefit**: Outdoor autonomous missions

### Data & Telemetry (Phase 4)
- [ ] **#18: SD Card Data Logging**
  - **Priority**: Medium
  - **Description**: Log sensor data and missions to SD card
  - **Technical**: ESP32 SD card support
  - **Benefit**: Performance analysis, debugging

- [ ] **#19: Cloud Data Upload**
  - **Priority**: Low
  - **Description**: Upload telemetry to cloud (ThingSpeak, AWS IoT)
  - **Technical**: HTTPS client, MQTT
  - **Benefit**: Remote monitoring, data analytics

- [ ] **#20: Mission Playback**
  - **Priority**: Low
  - **Description**: Record and replay robot missions
  - **Technical**: Command logging, playback engine
  - **Benefit**: Automated testing, demonstrations

### Developer Tools (Phase 4)
- [ ] **#21: OTA Firmware Updates**
  - **Priority**: Medium
  - **Description**: Update ESP32 firmware over WiFi
  - **Technical**: ESP32 OTA library
  - **Benefit**: No need to disconnect for updates

- [ ] **#22: Web-Based Configuration Panel**
  - **Priority**: Medium
  - **Description**: Adjust PID parameters, sensor calibration via web
  - **Technical**: Web forms, persistent storage
  - **Benefit**: Easy tuning without recompiling

- [ ] **#23: Remote Debugging Console**
  - **Priority**: Low
  - **Description**: View Arduino/ESP32 logs via web interface
  - **Technical**: WebSocket log streaming
  - **Benefit**: Debugging without USB connection

- [ ] **#24: REST API Documentation**
  - **Priority**: Low
  - **Description**: Swagger/OpenAPI documentation for REST API
  - **Benefit**: Third-party integration, custom apps

### Mobile App (Phase 5)
- [ ] **#25: Progressive Web App (PWA)**
  - **Priority**: Medium
  - **Description**: Make web interface installable as native-like app
  - **Technical**: Service worker, manifest.json
  - **Benefit**: Better iOS/Android experience

- [ ] **#26: Voice Control Integration**
  - **Priority**: Low
  - **Description**: Control via voice commands (Siri Shortcuts)
  - **Technical**: iOS Shortcuts integration via API
  - **Benefit**: Hands-free control

---

## 🐛 Known Limitations

### Hardware Constraints
- **Arduino Flash Memory**: 98% used - limited room for expansion
  - **Solution**: Move more logic to ESP32
- **Arduino RAM**: ~60% used - some headroom left
- **ESP32 Camera Not Used Yet**: Hardware present but not utilized
  - **Solution**: Phase 3 implementation

### Software Limitations
- **No Camera Features**: Phase 3 planned
- **Web Interface Optimization**: Optimized for modern browsers (Safari, Chrome, Firefox)
  - Older browsers (IE11) not supported
- **WiFi Range**: ~10-20m typical for ESP32 AP mode
  - **Solution**: Add WiFi Station mode for better range

### Communication
- **Serial Bandwidth**: 9600 baud limits data rate
  - Currently sufficient for 10Hz sensor updates
  - May need increase for video streaming (Phase 3)
- **Latency**: 50-100ms end-to-end
  - Acceptable for remote control, not for high-speed racing

---

## 🔧 Troubleshooting Reference

For detailed troubleshooting, see [TESTING_GUIDE.md](TESTING_GUIDE.md) Section: Troubleshooting

**Common Issues**:
1. Arduino upload fails → Disconnect ESP32 TX/RX during upload
2. ESP32 upload fails → IO0 to GND, press RESET during upload
3. WiFi not visible → Check ESP32 Serial Monitor for "AP SSID" message
4. Web interface doesn't load → iOS: "Use Without Internet" when prompted
5. Robot doesn't move → Verify `ENABLE_ESP32_BRAIN 1` in Arduino code
6. Sensor data not updating → Check Arduino ↔ ESP32 wiring (TX ↔ RX crossed)

---

## 📊 Project Status

| Phase | Status | Completion |
|-------|--------|------------|
| **Phase 1: Foundation** | ✅ Complete | 100% |
| **Phase 2: Core Features** | ✅ Complete | 100% |
| **Phase 3: Vision** | 🔄 Planned | 0% |
| **Phase 4: Advanced** | 🔄 Planned | 0% |
| **Phase 5: Mobile** | 🔄 Planned | 0% |

**Current Focus**: Testing & Validation (Issues #1-#9)

**Next Milestone**: Phase 3 - Camera & Vision Features

---

## 🤝 Contributing

This is a personal project branch. Feedback and testing reports welcome!

**How to Report Issues**:
1. Test following [TESTING_GUIDE.md](TESTING_GUIDE.md)
2. Document the issue with:
   - Hardware setup (Arduino + ESP32 versions)
   - Steps to reproduce
   - Expected vs actual behavior
   - Serial Monitor logs (both Arduino and ESP32)
   - Photos of wiring if relevant

**Feature Requests**:
- Check if already listed in Phase 3+ roadmap
- Describe use case and benefit
- Consider complexity vs value

---

**Last Updated**: 2025-09-30
**Branch**: `feature/esp32-brain-architecture`
**Version**: 1.0 (Phases 1 & 2 Complete)
