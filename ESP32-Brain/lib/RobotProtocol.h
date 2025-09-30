/*
 * RobotProtocol.h - Shared communication protocol for Arduino ↔ ESP32
 * Part of ESP32 Brain Architecture
 *
 * This file is shared between Arduino and ESP32 code
 * Binary protocol for compact, fast communication over Serial
 */

#pragma once
#include <Arduino.h>

// Protocol Constants
#define PROTOCOL_START_BYTE    0xAA
#define PROTOCOL_MAX_DATA_SIZE 32

// Message Types: ESP32 → Arduino (Commands)
enum ESP32Command : uint8_t {
    CMD_MOTOR_CONTROL      = 0x01,  // [dir, speedL, speedR]
    CMD_SERVO_CONTROL      = 0x02,  // [servo_id, angle]
    CMD_LED_CONTROL        = 0x03,  // [r, g, b]
    CMD_SET_MODE           = 0x04,  // [mode]
    CMD_EMERGENCY_STOP     = 0x05,  // []
    CMD_REQUEST_SENSORS    = 0x06,  // []
    CMD_PING               = 0x07,  // [] - Heartbeat
};

// Message Types: Arduino → ESP32 (Responses)
enum ArduinoResponse : uint8_t {
    RSP_SENSOR_DATA       = 0x81,  // [14 bytes sensor packet]
    RSP_ERROR             = 0x82,  // [error_code]
    RSP_STATUS            = 0x83,  // [status_byte]
    RSP_PONG              = 0x84,  // [] - Heartbeat response
};

// Motor Direction Encoding
enum MotorDirection : uint8_t {
    DIR_FORWARD       = 0x00,
    DIR_BACKWARD      = 0x01,
    DIR_LEFT          = 0x02,
    DIR_RIGHT         = 0x03,
    DIR_LEFT_FORWARD  = 0x04,
    DIR_LEFT_BACKWARD = 0x05,
    DIR_RIGHT_FORWARD = 0x06,
    DIR_RIGHT_BACKWARD = 0x07,
    DIR_STOP          = 0x08,
};

// Operating Modes
enum OperatingMode : uint8_t {
    MODE_STANDBY          = 0x00,
    MODE_MANUAL           = 0x01,
    MODE_LINE_FOLLOW      = 0x02,
    MODE_OBSTACLE_AVOID   = 0x03,
    MODE_OBJECT_FOLLOW    = 0x04,
    MODE_AUTONOMOUS       = 0x05,
};

// Error Codes
enum ErrorCode : uint8_t {
    ERR_NONE              = 0x00,
    ERR_INVALID_COMMAND   = 0x01,
    ERR_CHECKSUM_FAIL     = 0x02,
    ERR_TIMEOUT           = 0x03,
    ERR_MOTOR_FAULT       = 0x04,
    ERR_SENSOR_FAULT      = 0x05,
    ERR_LOW_BATTERY       = 0x06,
};

// Sensor Data Packet Structure (14 bytes total)
struct __attribute__((packed)) SensorPacket {
    uint16_t ultrasonic_cm;        // 0-400 cm
    uint16_t line_left;            // 0-1023
    uint16_t line_middle;          // 0-1023
    uint16_t line_right;           // 0-1023
    uint16_t voltage_mv;           // mV
    int16_t  imu_pitch;            // -180 to +180 degrees * 10
    int16_t  imu_roll;             // -180 to +180 degrees * 10
    uint8_t  mode;                 // Current operating mode
    uint8_t  flags;                // Status flags (bit field)
};

// Status Flags (bit field)
#define STATUS_FLAG_MOTORS_OK      (1 << 0)
#define STATUS_FLAG_SENSORS_OK     (1 << 1)
#define STATUS_FLAG_BATTERY_OK     (1 << 2)
#define STATUS_FLAG_ON_GROUND      (1 << 3)
#define STATUS_FLAG_EMERGENCY_STOP (1 << 4)

// Message Structure
struct __attribute__((packed)) ProtocolMessage {
    uint8_t start;                            // 0xAA
    uint8_t type;                             // Message type
    uint8_t length;                           // Data length (0-32)
    uint8_t data[PROTOCOL_MAX_DATA_SIZE];     // Payload
    uint8_t checksum;                         // Simple checksum
};

// Protocol Helper Class
class RobotProtocol {
public:
    RobotProtocol(Stream* serial) : _serial(serial), _lastError(ERR_NONE) {}

    // Calculate checksum
    static uint8_t calculateChecksum(const ProtocolMessage* msg) {
        uint8_t sum = msg->start + msg->type + msg->length;
        for (uint8_t i = 0; i < msg->length; i++) {
            sum += msg->data[i];
        }
        return ~sum; // One's complement
    }

    // Verify checksum
    static bool verifyChecksum(const ProtocolMessage* msg) {
        return calculateChecksum(msg) == msg->checksum;
    }

    // Send a message
    bool sendMessage(uint8_t type, const uint8_t* data = nullptr, uint8_t length = 0) {
        if (length > PROTOCOL_MAX_DATA_SIZE) return false;

        ProtocolMessage msg;
        msg.start = PROTOCOL_START_BYTE;
        msg.type = type;
        msg.length = length;

        if (data && length > 0) {
            memcpy(msg.data, data, length);
        }

        msg.checksum = calculateChecksum(&msg);

        // Send message
        size_t written = _serial->write((uint8_t*)&msg, 3 + length + 1);
        return written == (3 + length + 1);
    }

    // Receive a message (non-blocking)
    bool receiveMessage(ProtocolMessage* msg, uint16_t timeout_ms = 100) {
        unsigned long start = millis();

        // Wait for start byte
        while (millis() - start < timeout_ms) {
            if (_serial->available() >= 1) {
                uint8_t byte = _serial->read();
                if (byte == PROTOCOL_START_BYTE) {
                    msg->start = byte;
                    break;
                }
            }
        }

        if (msg->start != PROTOCOL_START_BYTE) {
            _lastError = ERR_TIMEOUT;
            return false;
        }

        // Read type and length
        start = millis();
        while (millis() - start < timeout_ms && _serial->available() < 2);

        if (_serial->available() < 2) {
            _lastError = ERR_TIMEOUT;
            return false;
        }

        msg->type = _serial->read();
        msg->length = _serial->read();

        if (msg->length > PROTOCOL_MAX_DATA_SIZE) {
            _lastError = ERR_INVALID_COMMAND;
            return false;
        }

        // Read data + checksum
        uint8_t total = msg->length + 1;
        start = millis();
        while (millis() - start < timeout_ms && _serial->available() < total);

        if (_serial->available() < total) {
            _lastError = ERR_TIMEOUT;
            return false;
        }

        if (msg->length > 0) {
            _serial->readBytes(msg->data, msg->length);
        }
        msg->checksum = _serial->read();

        // Verify checksum
        if (!verifyChecksum(msg)) {
            _lastError = ERR_CHECKSUM_FAIL;
            return false;
        }

        _lastError = ERR_NONE;
        return true;
    }

    // Get last error
    ErrorCode getLastError() const { return _lastError; }

    // Clear serial buffer
    void clearBuffer() {
        while (_serial->available()) {
            _serial->read();
        }
    }

private:
    Stream* _serial;
    ErrorCode _lastError;
};