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

// Forward declarations of external functions
extern void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

class ESP32CommunicationHandler {
private:
    RobotProtocol protocol;
    unsigned long lastSensorSend;
    unsigned long lastHeartbeat;
    bool esp32Connected;

    static const uint16_t SENSOR_SEND_INTERVAL = 100;  // Send sensors every 100ms (10Hz)
    static const uint16_t HEARTBEAT_TIMEOUT = 2000;    // 2 seconds without heartbeat = disconnected

public:
    ESP32CommunicationHandler() :
        protocol(&Serial),
        lastSensorSend(0),
        lastHeartbeat(0),
        esp32Connected(false) {}

    void init() {
        // Serial already initialized by ApplicationFunctionSet_Init()
        // Just mark as potentially connected
        esp32Connected = false;
    }

    // Process incoming commands from ESP32
    void processCommands() {
        ProtocolMessage msg;

        // Non-blocking receive - check if message available
        if (protocol.receiveMessage(&msg, 5)) {  // 5ms timeout
            esp32Connected = true;
            lastHeartbeat = millis();

            handleCommand(&msg);
        }

        // Check for timeout
        if (esp32Connected && (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT)) {
            esp32Connected = false;
            // Optional: Switch to safe mode when ESP32 disconnects
        }
    }

    // Send sensor data to ESP32
    void sendSensorData() {
        if (!esp32Connected) return;  // Don't send if not connected

        if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
            SensorPacket sensors = collectSensorData();

            protocol.sendMessage(RSP_SENSOR_DATA,
                                (uint8_t*)&sensors,
                                sizeof(SensorPacket));

            lastSensorSend = millis();
        }
    }

    bool isESP32Connected() const { return esp32Connected; }

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
                break;
            }

            default:
                sendError(ERR_INVALID_COMMAND);
                break;
        }
    }

    void handleMotorCommand(uint8_t dir, uint8_t speedL, uint8_t speedR) {
        extern DeviceDriverSet_Motor AppMotor;
        extern Application_xxx Application_SmartRobotCarxxx0;

        // Set to manual/rocker mode when receiving ESP32 commands
        Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;

        // Average speed for unified control
        uint8_t speed = (speedL + speedR) / 2;

        SmartRobotCarMotionControl motion;

        switch (dir) {
            case DIR_FORWARD:
                motion = Forward;
                break;
            case DIR_BACKWARD:
                motion = Backward;
                break;
            case DIR_LEFT:
                motion = Left;
                break;
            case DIR_RIGHT:
                motion = Right;
                break;
            case DIR_LEFT_FORWARD:
                motion = LeftForward;
                break;
            case DIR_LEFT_BACKWARD:
                motion = LeftBackward;
                break;
            case DIR_RIGHT_FORWARD:
                motion = RightForward;
                break;
            case DIR_RIGHT_BACKWARD:
                motion = RightBackward;
                break;
            case DIR_STOP:
            default:
                motion = stop_it;
                speed = 0;
                break;
        }

        // Use existing motor control function
        ApplicationFunctionSet_SmartRobotCarMotionControl(motion, speed);
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
    }
};