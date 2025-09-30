/*
 * ESP32 Brain - Main Controller
 * Part of ELEGOO Smart Robot Car V4.0 ESP32 Brain Architecture
 *
 * This is the "brain" of the robot - handles:
 * - WiFi AP & Web Server
 * - Camera streaming
 * - Vision processing
 * - Path planning
 * - Communication with Arduino Uno
 *
 * Hardware: ESP32-WROVER-CAM (AI Thinker)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "lib/RobotProtocol.h"

// WiFi AP Configuration
const char* AP_SSID = "ELEGOO-Robot";
const char* AP_PASSWORD = "elegoo123";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// Serial connection to Arduino
// Note: Serial is used for debugging (USB), Serial2 for Arduino communication
#define ARDUINO_SERIAL Serial2
#define ARDUINO_BAUD 9600
#define ARDUINO_RX_PIN 16  // GPIO16 (U2RXD)
#define ARDUINO_TX_PIN 17  // GPIO17 (U2TXD)

// Web Server
WebServer server(80);

// Protocol handler for Arduino communication (will be initialized in setup)
RobotProtocol* protocol = nullptr;

// Current state
struct RobotState {
    SensorPacket sensors;
    bool arduinoConnected;
    unsigned long lastSensorUpdate;
    OperatingMode currentMode;
    uint8_t motorSpeed;
} robotState;

// Function prototypes
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleAPI();
void handleNotFound();
void processArduinoMessages();
void sendMotorCommand(MotorDirection dir, uint8_t speed);
String generateWebInterface();

void setup() {
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(1000);

    Serial.println("=================================");
    Serial.println("ELEGOO Robot - ESP32 Brain v1.0");
    Serial.println("=================================");

    // Initialize state
    robotState.arduinoConnected = false;
    robotState.lastSensorUpdate = 0;
    robotState.currentMode = MODE_STANDBY;
    robotState.motorSpeed = 150;
    memset(&robotState.sensors, 0, sizeof(SensorPacket));

    // Setup WiFi AP
    setupWiFi();

    // Setup Web Server
    setupWebServer();

    // Start Arduino communication on Serial2
    Serial.println("[Serial] Initializing Arduino communication on Serial2...");
    ARDUINO_SERIAL.begin(ARDUINO_BAUD, SERIAL_8N1, ARDUINO_RX_PIN, ARDUINO_TX_PIN);
    delay(100);

    // Initialize protocol handler
    protocol = new RobotProtocol(&ARDUINO_SERIAL);

    Serial.println("\n✓ ESP32 Brain Ready!");
    Serial.print("✓ Web Interface: http://");
    Serial.print(WiFi.softAPIP());
    Serial.println("/");
    Serial.println("✓ mDNS: http://elegoo-robot.local");
    Serial.print("✓ Arduino Serial: RX=GPIO");
    Serial.print(ARDUINO_RX_PIN);
    Serial.print(", TX=GPIO");
    Serial.println(ARDUINO_TX_PIN);
}

void loop() {
    // Handle web requests
    server.handleClient();

    // Process messages from Arduino
    processArduinoMessages();

    // Send heartbeat every second
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 1000) {
        if (protocol) {
            protocol->sendMessage(CMD_PING, nullptr, 0);
        }
        lastHeartbeat = millis();
    }

    // Request sensor data every 100ms
    static unsigned long lastSensorRequest = 0;
    if (millis() - lastSensorRequest > 100) {
        if (protocol) {
            protocol->sendMessage(CMD_REQUEST_SENSORS, nullptr, 0);
        }
        lastSensorRequest = millis();
    }
}

void setupWiFi() {
    Serial.println("\n[WiFi] Setting up Access Point...");

    // Configure AP
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    delay(100);

    Serial.print("[WiFi] AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());

    // Setup mDNS
    if (MDNS.begin("elegoo-robot")) {
        Serial.println("[mDNS] Responder started: elegoo-robot.local");
    }
}

void setupWebServer() {
    Serial.println("[WebServer] Setting up routes...");

    // Main web interface
    server.on("/", HTTP_GET, handleRoot);

    // API endpoints
    server.on("/api/move", HTTP_POST, []() {
        if (!server.hasArg("direction") || !server.hasArg("speed")) {
            server.send(400, "application/json", "{\"error\":\"Missing parameters\"}");
            return;
        }

        String direction = server.arg("direction");
        int speed = server.arg("speed").toInt();

        MotorDirection dir = DIR_STOP;
        if (direction == "forward") dir = DIR_FORWARD;
        else if (direction == "backward") dir = DIR_BACKWARD;
        else if (direction == "left") dir = DIR_LEFT;
        else if (direction == "right") dir = DIR_RIGHT;
        else if (direction == "stop") dir = DIR_STOP;

        sendMotorCommand(dir, (uint8_t)speed);
        server.send(200, "application/json", "{\"status\":\"ok\"}");
    });

    server.on("/api/sensors", HTTP_GET, []() {
        char json[256];
        snprintf(json, sizeof(json),
            "{\"ultrasonic\":%d,\"lineL\":%d,\"lineM\":%d,\"lineR\":%d,\"voltage\":%.2f,\"mode\":%d}",
            robotState.sensors.ultrasonic_cm,
            robotState.sensors.line_left,
            robotState.sensors.line_middle,
            robotState.sensors.line_right,
            robotState.sensors.voltage_mv / 1000.0,
            robotState.sensors.mode
        );
        server.send(200, "application/json", json);
    });

    server.on("/api/mode", HTTP_POST, []() {
        if (!server.hasArg("mode")) {
            server.send(400, "application/json", "{\"error\":\"Missing mode\"}");
            return;
        }

        if (protocol) {
            int mode = server.arg("mode").toInt();
            uint8_t data = (uint8_t)mode;
            protocol->sendMessage(CMD_SET_MODE, &data, 1);
        }

        server.send(200, "application/json", "{\"status\":\"ok\"}");
    });

    server.on("/api/stop", HTTP_POST, []() {
        if (protocol) {
            protocol->sendMessage(CMD_EMERGENCY_STOP, nullptr, 0);
        }
        server.send(200, "application/json", "{\"status\":\"ok\"}");
    });

    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("[WebServer] HTTP server started on port 80");
}

void handleRoot() {
    server.send(200, "text/html", generateWebInterface());
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

void processArduinoMessages() {
    if (!protocol) return;

    ProtocolMessage msg;

    if (protocol->receiveMessage(&msg, 5)) {
        robotState.arduinoConnected = true;

        switch (msg.type) {
            case RSP_SENSOR_DATA:
                if (msg.length == sizeof(SensorPacket)) {
                    memcpy(&robotState.sensors, msg.data, sizeof(SensorPacket));
                    robotState.lastSensorUpdate = millis();
                }
                break;

            case RSP_ERROR:
                Serial.print("[Arduino] Error code: ");
                Serial.println(msg.data[0]);
                break;

            case RSP_PONG:
                // Heartbeat received
                break;
        }
    }

    // Check connection timeout
    if (millis() - robotState.lastSensorUpdate > 2000) {
        robotState.arduinoConnected = false;
    }
}

void sendMotorCommand(MotorDirection dir, uint8_t speed) {
    if (!protocol) return;

    uint8_t data[3];
    data[0] = (uint8_t)dir;
    data[1] = speed;  // speedL
    data[2] = speed;  // speedR

    protocol->sendMessage(CMD_MOTOR_CONTROL, data, 3);
}

String generateWebInterface() {
    // Web interface will be in next file (too large for single file)
    // For now, return a simple placeholder
    return R"html(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>ELEGOO Robot Control</title>
    <style>
        body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; background: #1a1a1a; color: #fff; }
        .container { max-width: 600px; margin: 0 auto; padding: 20px; }
        h1 { text-align: center; color: #4CAF50; }
        .status { padding: 15px; background: #2a2a2a; border-radius: 8px; margin-bottom: 20px; }
        .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin: 20px 0; }
        button { padding: 20px; font-size: 18px; border: none; border-radius: 8px; background: #4CAF50; color: white; cursor: pointer; touch-action: manipulation; }
        button:active { background: #45a049; }
        button.stop { background: #f44336; grid-column: span 3; }
        .sensor-data { background: #2a2a2a; padding: 15px; border-radius: 8px; margin-top: 20px; }
        .sensor-item { display: flex; justify-content: space-between; padding: 8px 0; border-bottom: 1px solid #444; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 ELEGOO Robot</h1>

        <div class="status">
            <strong>Status:</strong> <span id="status">Connecting...</span><br>
            <strong>Mode:</strong> <span id="mode">Standby</span>
        </div>

        <div class="controls">
            <div></div>
            <button ontouchstart="move('forward')" ontouchend="stop()">▲</button>
            <div></div>
            <button ontouchstart="move('left')" ontouchend="stop()">◀</button>
            <button ontouchstart="stop()">⬛</button>
            <button ontouchstart="move('right')" ontouchend="stop()">▶</button>
            <div></div>
            <button ontouchstart="move('backward')" ontouchend="stop()">▼</button>
            <div></div>
            <button class="stop" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
        </div>

        <div class="sensor-data">
            <h3>Sensor Data</h3>
            <div class="sensor-item"><span>Distance:</span><span id="distance">--</span></div>
            <div class="sensor-item"><span>Voltage:</span><span id="voltage">--</span></div>
            <div class="sensor-item"><span>Line L/M/R:</span><span id="line">--</span></div>
        </div>
    </div>

    <script>
        function move(direction) {
            fetch('/api/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: 'direction=' + direction + '&speed=150'
            });
        }

        function stop() {
            fetch('/api/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: 'direction=stop&speed=0'
            });
        }

        function emergencyStop() {
            fetch('/api/stop', {method: 'POST'});
        }

        function updateSensors() {
            fetch('/api/sensors')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('distance').textContent = data.ultrasonic + ' cm';
                    document.getElementById('voltage').textContent = data.voltage.toFixed(1) + ' V';
                    document.getElementById('line').textContent = data.lineL + ' / ' + data.lineM + ' / ' + data.lineR;
                    document.getElementById('status').textContent = 'Connected ✓';
                })
                .catch(() => {
                    document.getElementById('status').textContent = 'Disconnected ✗';
                });
        }

        setInterval(updateSensors, 500);
        updateSensors();
    </script>
</body>
</html>
)html";
}