/*
 * ESP32CommunicationHandler.h - Handle communication with ESP32 Brain
 * Part of ESP32 Brain Architecture
 *
 * This handler runs in parallel with existing control methods
 * Allows both old (BLE app, IR) and new (ESP32 web) control to coexist
 */

#pragma once
#include <Arduino.h>
#include "RobotProtocol.h"
#include "ApplicationFunctionSet_xxx0.h"

// Forward declarations - function signature defined in ApplicationFunctionSet_xxx0.cpp
// We can't forward declare it here because the enum SmartRobotCarMotionControl
// is defined in the .cpp file. Instead, we'll declare it in the function where needed.

// Connection state tracking
enum ESP32ConnectionState {
    STATE_DISCONNECTED = 0,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_TIMEOUT_WARNING
};

// Diagnostic counters
struct ESP32Diagnostics {
    uint16_t messagesReceived;
    uint16_t messagesSent;
    uint16_t crcErrors;
    uint16_t timeouts;
    uint16_t invalidCommands;
    uint32_t totalLoopTime;
    uint16_t loopCount;
};

class ESP32CommunicationHandler {
private:
    RobotProtocol protocol;
    unsigned long lastSensorSend;
    unsigned long lastHeartbeat;
    unsigned long lastStateChange;
    unsigned long lastDiagnosticSend;
    ESP32ConnectionState connectionState;

    // Previous sensor values for change detection
    uint16_t lastUltrasonic;
    uint8_t lastLineL, lastLineM, lastLineR;
    uint16_t lastVoltage;

    // Diagnostics
    ESP32Diagnostics diagnostics;

    static const uint16_t SENSOR_SEND_INTERVAL = 100;  // Send sensors every 100ms (10Hz)
    static const uint16_t HEARTBEAT_TIMEOUT = 2000;    // 2 seconds without heartbeat = disconnected
    static const uint16_t STATE_WARNING_TIME = 1500;   // Warning before timeout
    static const uint16_t DIAGNOSTIC_INTERVAL = 5000;  // Send diagnostics every 5s
    static const uint16_t SENSOR_CHANGE_THRESHOLD = 2; // Only send if changed significantly

public:
    ESP32CommunicationHandler() :
        protocol(&Serial),
        lastSensorSend(0),
        lastHeartbeat(0),
        lastStateChange(0),
        lastDiagnosticSend(0),
        connectionState(STATE_DISCONNECTED),
        lastUltrasonic(0),
        lastLineL(0), lastLineM(0), lastLineR(0),
        lastVoltage(0) {
        // Initialize diagnostics
        memset(&diagnostics, 0, sizeof(ESP32Diagnostics));
    }

    void init() {
        // Serial already initialized by ApplicationFunctionSet_Init()
        changeState(STATE_DISCONNECTED);
    }

    // Process incoming commands from ESP32
    void processCommands() {
        unsigned long loopStart = millis();
        ProtocolMessage msg;

        // Non-blocking receive - check if message available
        if (protocol.receiveMessage(&msg, 5)) {  // 5ms timeout
            diagnostics.messagesReceived++;
            lastHeartbeat = millis();

            // State transition: got first message
            if (connectionState == STATE_DISCONNECTED) {
                changeState(STATE_CONNECTING);
            } else if (connectionState == STATE_CONNECTING) {
                changeState(STATE_CONNECTED);
            } else if (connectionState == STATE_TIMEOUT_WARNING) {
                changeState(STATE_CONNECTED);  // Recovered!
            }

            handleCommand(&msg);
        }

        // Check connection health
        updateConnectionState();

        // Update performance metrics
        unsigned long loopTime = millis() - loopStart;
        diagnostics.totalLoopTime += loopTime;
        diagnostics.loopCount++;

        // Send periodic diagnostics
        if (connectionState == STATE_CONNECTED &&
            millis() - lastDiagnosticSend >= DIAGNOSTIC_INTERVAL) {
            sendDiagnostics();
            lastDiagnosticSend = millis();
        }
    }

    // Send sensor data to ESP32 (with change detection optimization)
    void sendSensorData() {
        if (connectionState != STATE_CONNECTED) return;  // Only send when fully connected

        if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
            SensorPacket sensors = collectSensorData();

            // Validate sensor data before sending
            if (validateSensorData(&sensors)) {
                // Check if significant change occurred
                if (sensorDataChanged(&sensors)) {
                    protocol.sendMessage(RSP_SENSOR_DATA,
                                        (uint8_t*)&sensors,
                                        sizeof(SensorPacket));
                    diagnostics.messagesSent++;

                    // Update last values
                    lastUltrasonic = sensors.ultrasonic_cm;
                    lastLineL = sensors.line_left;
                    lastLineM = sensors.line_middle;
                    lastLineR = sensors.line_right;
                    lastVoltage = sensors.voltage_mv;
                }
            }

            lastSensorSend = millis();
        }
    }

    // Public status accessors
    bool isESP32Connected() const { return connectionState == STATE_CONNECTED; }
    ESP32ConnectionState getConnectionState() const { return connectionState; }
    const ESP32Diagnostics* getDiagnostics() const { return &diagnostics; }

private:
    void handleCommand(ProtocolMessage* msg) {
        extern ApplicationFunctionSet Application_FunctionSet;
        extern Application_xxx Application_SmartRobotCarxxx0;

        switch (msg->type) {
            case CMD_MOTOR_CONTROL: {
                if (msg->length >= 3) {
                    uint8_t dir = msg->data[0];
                    uint8_t speedL = msg->data[1];
                    uint8_t speedR = msg->data[2];

                    // Convert to motor control
                    handleMotorCommand(dir, speedL, speedR);
                }
                break;
            }

            case CMD_SERVO_CONTROL: {
                if (msg->length >= 2) {
                    uint8_t servo_id = msg->data[0];
                    uint8_t angle = msg->data[1];

                    extern DeviceDriverSet_Servo AppServo;
                    if (servo_id == 0) {
                        AppServo.DeviceDriverSet_Servo_control(angle);
                    }
                }
                break;
            }

            case CMD_LED_CONTROL: {
                if (msg->length >= 3) {
                    uint8_t r = msg->data[0];
                    uint8_t g = msg->data[1];
                    uint8_t b = msg->data[2];

                    extern DeviceDriverSet_RBGLED AppRBG_LED;
                    AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, r, g, b);
                }
                break;
            }

            case CMD_SET_MODE: {
                if (msg->length >= 1) {
                    uint8_t mode = msg->data[0];
                    setOperatingMode(mode);
                }
                break;
            }

            case CMD_EMERGENCY_STOP: {
                extern DeviceDriverSet_Motor AppMotor;
                AppMotor.DeviceDriverSet_Motor_control(
                    direction_void, 0, direction_void, 0, control_enable);
                break;
            }

            case CMD_REQUEST_SENSORS: {
                // Force immediate sensor send
                lastSensorSend = 0;
                break;
            }

            case CMD_PING: {
                // Respond with PONG
                protocol.sendMessage(RSP_PONG, nullptr, 0);
                diagnostics.messagesSent++;
                break;
            }

            case CMD_GET_DIAGNOSTICS: {
                // Force immediate diagnostic send
                sendDiagnostics();
                break;
            }

            case CMD_RESET_DIAGNOSTICS: {
                // Reset diagnostic counters
                memset(&diagnostics, 0, sizeof(ESP32Diagnostics));
                protocol.sendMessage(RSP_ACK, nullptr, 0);
                diagnostics.messagesSent++;
                break;
            }

            case CMD_CALIBRATE_SENSORS: {
                // Trigger sensor calibration (if implemented)
                extern ApplicationFunctionSet Application_FunctionSet;
                // Future: Add calibration routine
                protocol.sendMessage(RSP_ACK, nullptr, 0);
                diagnostics.messagesSent++;
                break;
            }

            default:
                diagnostics.invalidCommands++;
                sendError(ERR_INVALID_COMMAND);
                break;
        }
    }

    void handleMotorCommand(uint8_t dir, uint8_t speedL, uint8_t speedR) {
        extern DeviceDriverSet_Motor AppMotor;
        extern Application_xxx Application_SmartRobotCarxxx0;
        extern ApplicationFunctionSet Application_FunctionSet;

        // Set to manual/rocker mode when receiving ESP32 commands
        Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;

        // Average speed for unified control
        uint8_t speed = (speedL + speedR) / 2;

        // Map protocol direction to motor control
        // We use the Application_FunctionSet method directly
        switch (dir) {
            case DIR_FORWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, speed);
                break;
            case DIR_BACKWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, speed);
                break;
            case DIR_LEFT:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(Left, speed);
                break;
            case DIR_RIGHT:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(Right, speed);
                break;
            case DIR_LEFT_FORWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(LeftForward, speed);
                break;
            case DIR_LEFT_BACKWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(LeftBackward, speed);
                break;
            case DIR_RIGHT_FORWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(RightForward, speed);
                break;
            case DIR_RIGHT_BACKWARD:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(RightBackward, speed);
                break;
            case DIR_STOP:
            default:
                Application_FunctionSet.ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
                break;
        }
    }

    void setOperatingMode(uint8_t mode) {
        extern Application_xxx Application_SmartRobotCarxxx0;

        switch (mode) {
            case MODE_STANDBY:
                Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
                break;
            case MODE_MANUAL:
                Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
                break;
            case MODE_LINE_FOLLOW:
                Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
                break;
            case MODE_OBSTACLE_AVOID:
                Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
                break;
            case MODE_OBJECT_FOLLOW:
                Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
                break;
            default:
                break;
        }
    }

    SensorPacket collectSensorData() {
        extern ApplicationFunctionSet Application_FunctionSet;
        extern Application_xxx Application_SmartRobotCarxxx0;

        SensorPacket packet;

        packet.ultrasonic_cm = Application_FunctionSet.UltrasoundData_cm;
        packet.line_left = Application_FunctionSet.TrackingData_L;
        packet.line_middle = Application_FunctionSet.TrackingData_M;
        packet.line_right = Application_FunctionSet.TrackingData_R;
        packet.voltage_mv = (uint16_t)(Application_FunctionSet.VoltageData * 1000);

        // IMU data (multiply by 10 for decimal precision)
        packet.imu_pitch = (int16_t)(Application_FunctionSet.Pitch * 10);
        packet.imu_roll = (int16_t)(Application_FunctionSet.Roll * 10);

        packet.mode = (uint8_t)Application_SmartRobotCarxxx0.Functional_Mode;

        // Build status flags
        packet.flags = 0;
        packet.flags |= STATUS_FLAG_MOTORS_OK;      // Assume motors OK
        packet.flags |= STATUS_FLAG_SENSORS_OK;     // Assume sensors OK

        if (Application_FunctionSet.VoltageData > 6.5) {
            packet.flags |= STATUS_FLAG_BATTERY_OK;
        }

        if (!Application_FunctionSet.Car_LeaveTheGround) {
            packet.flags |= STATUS_FLAG_ON_GROUND;
        }

        return packet;
    }

    void sendError(ErrorCode error) {
        uint8_t data = (uint8_t)error;
        protocol.sendMessage(RSP_ERROR, &data, 1);
        diagnostics.messagesSent++;
    }

    // State management
    void changeState(ESP32ConnectionState newState) {
        if (connectionState != newState) {
            connectionState = newState;
            lastStateChange = millis();
        }
    }

    void updateConnectionState() {
        unsigned long timeSinceHeartbeat = millis() - lastHeartbeat;

        switch (connectionState) {
            case STATE_DISCONNECTED:
                // Stay disconnected until first message
                break;

            case STATE_CONNECTING:
                // Need second message within timeout to confirm connection
                if (timeSinceHeartbeat > HEARTBEAT_TIMEOUT) {
                    changeState(STATE_DISCONNECTED);
                    diagnostics.timeouts++;
                }
                break;

            case STATE_CONNECTED:
                // Watch for timeout warning
                if (timeSinceHeartbeat > STATE_WARNING_TIME) {
                    changeState(STATE_TIMEOUT_WARNING);
                }
                break;

            case STATE_TIMEOUT_WARNING:
                // Full timeout - disconnect
                if (timeSinceHeartbeat > HEARTBEAT_TIMEOUT) {
                    changeState(STATE_DISCONNECTED);
                    diagnostics.timeouts++;
                    handleDisconnect();
                }
                break;
        }
    }

    void handleDisconnect() {
        // Optional: Safe actions when ESP32 disconnects
        // For example: stop motors, switch to standby mode

        // Could switch to standby, but let's allow IR/BLE to continue
        // extern Application_xxx Application_SmartRobotCarxxx0;
        // Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
    }

    // Sensor validation - check for plausible values
    bool validateSensorData(SensorPacket* packet) {
        // Ultrasonic: 2-400cm is valid range for HC-SR04
        if (packet->ultrasonic_cm > 400) {
            packet->ultrasonic_cm = 400;  // Clamp to max
        }

        // Line sensors: 0 or 1 (binary)
        // Already validated by hardware

        // Voltage: 6.0V - 8.4V reasonable for 2S LiPo
        if (packet->voltage_mv < 5000 || packet->voltage_mv > 9000) {
            // Out of expected range, but still send
            packet->flags &= ~STATUS_FLAG_BATTERY_OK;
        }

        // IMU: -900 to +900 (representing -90.0 to +90.0 degrees)
        if (packet->imu_pitch < -900) packet->imu_pitch = -900;
        if (packet->imu_pitch > 900) packet->imu_pitch = 900;
        if (packet->imu_roll < -900) packet->imu_roll = -900;
        if (packet->imu_roll > 900) packet->imu_roll = 900;

        return true;  // Always return true, we just clamp values
    }

    // Check if sensor data changed significantly
    bool sensorDataChanged(SensorPacket* packet) {
        // Always send on first call (lastUltrasonic == 0)
        if (lastUltrasonic == 0) return true;

        // Check for significant changes
        bool changed = false;

        // Ultrasonic: > 2cm change
        if (abs((int)packet->ultrasonic_cm - (int)lastUltrasonic) > SENSOR_CHANGE_THRESHOLD) {
            changed = true;
        }

        // Line sensors: any change
        if (packet->line_left != lastLineL ||
            packet->line_middle != lastLineM ||
            packet->line_right != lastLineR) {
            changed = true;
        }

        // Voltage: > 100mV change (0.1V)
        if (abs((int)packet->voltage_mv - (int)lastVoltage) > 100) {
            changed = true;
        }

        // Mode changes always sent
        // IMU changes sent if significant (future: add threshold)

        // Force send every 10th interval even if no change (heartbeat)
        static uint8_t forceSendCounter = 0;
        if (++forceSendCounter >= 10) {
            forceSendCounter = 0;
            changed = true;
        }

        return changed;
    }

    // Send diagnostic information
    void sendDiagnostics() {
        // Build diagnostic packet
        uint8_t diagData[16];
        uint16_t avgLoopTime = (diagnostics.loopCount > 0) ?
            (diagnostics.totalLoopTime / diagnostics.loopCount) : 0;

        // Pack diagnostic data
        diagData[0] = diagnostics.messagesReceived & 0xFF;
        diagData[1] = (diagnostics.messagesReceived >> 8) & 0xFF;
        diagData[2] = diagnostics.messagesSent & 0xFF;
        diagData[3] = (diagnostics.messagesSent >> 8) & 0xFF;
        diagData[4] = diagnostics.crcErrors & 0xFF;
        diagData[5] = (diagnostics.crcErrors >> 8) & 0xFF;
        diagData[6] = diagnostics.timeouts & 0xFF;
        diagData[7] = (diagnostics.timeouts >> 8) & 0xFF;
        diagData[8] = diagnostics.invalidCommands & 0xFF;
        diagData[9] = (diagnostics.invalidCommands >> 8) & 0xFF;
        diagData[10] = avgLoopTime & 0xFF;
        diagData[11] = (avgLoopTime >> 8) & 0xFF;
        diagData[12] = (uint8_t)connectionState;
        diagData[13] = 0;  // Reserved
        diagData[14] = 0;  // Reserved
        diagData[15] = 0;  // Reserved

        protocol.sendMessage(RSP_DIAGNOSTIC, diagData, 16);
        diagnostics.messagesSent++;
    }
};