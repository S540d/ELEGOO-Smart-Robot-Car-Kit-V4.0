#pragma once
#include <Arduino.h>

class SmoothMotionController {
private:
    uint8_t currentSpeed_L, currentSpeed_R, targetSpeed_L, targetSpeed_R;
    static const uint8_t ACCEL_STEP = 5;
    static const uint8_t UPDATE_INTERVAL = 20;
    static const uint16_t MIN_OBSTACLE_DISTANCE = 15;
    static const uint16_t SLOW_OBSTACLE_DISTANCE = 30;
    unsigned long lastUpdate;
    bool emergencyStopActive;

public:
    SmoothMotionController() : currentSpeed_L(0), currentSpeed_R(0),
        targetSpeed_L(0), targetSpeed_R(0), lastUpdate(0), emergencyStopActive(false) {}

    void setTargetSpeed(uint8_t speedL, uint8_t speedR) {
        if (speedL == 0 && speedR == 0) emergencyStopActive = false;
        targetSpeed_L = speedL;
        targetSpeed_R = speedR;
    }

    void emergencyStop() {
        targetSpeed_L = targetSpeed_R = currentSpeed_L = currentSpeed_R = 0;
        emergencyStopActive = true;
    }

    bool update(uint8_t &outputSpeed_L, uint8_t &outputSpeed_R) {
        if (emergencyStopActive) {
            outputSpeed_L = 0;
            outputSpeed_R = 0;
            return true;
        }

        unsigned long now = millis();
        if (now - lastUpdate < UPDATE_INTERVAL) {
            outputSpeed_L = currentSpeed_L;
            outputSpeed_R = currentSpeed_R;
            return false;
        }
        lastUpdate = now;

        if (currentSpeed_L < targetSpeed_L) currentSpeed_L = min(currentSpeed_L + ACCEL_STEP, targetSpeed_L);
        else if (currentSpeed_L > targetSpeed_L) currentSpeed_L = max((int)currentSpeed_L - ACCEL_STEP, (int)targetSpeed_L);

        if (currentSpeed_R < targetSpeed_R) currentSpeed_R = min(currentSpeed_R + ACCEL_STEP, targetSpeed_R);
        else if (currentSpeed_R > targetSpeed_R) currentSpeed_R = max((int)currentSpeed_R - ACCEL_STEP, (int)targetSpeed_R);

        outputSpeed_L = currentSpeed_L;
        outputSpeed_R = currentSpeed_R;
        return true;
    }

    uint8_t adjustForObstacle(uint16_t distance_cm, uint8_t requestedSpeed) {
        if (distance_cm == 0 || distance_cm > 400) return requestedSpeed;
        if (distance_cm < MIN_OBSTACLE_DISTANCE) { emergencyStop(); return 0; }
        if (distance_cm < SLOW_OBSTACLE_DISTANCE) {
            uint16_t range = SLOW_OBSTACLE_DISTANCE - MIN_OBSTACLE_DISTANCE;
            uint8_t adjustedSpeed = (requestedSpeed * (distance_cm - MIN_OBSTACLE_DISTANCE)) / range;
            return max(adjustedSpeed, (uint8_t)30);
        }
        return requestedSpeed;
    }

};

// Global instance
SmoothMotionController smoothMotion;