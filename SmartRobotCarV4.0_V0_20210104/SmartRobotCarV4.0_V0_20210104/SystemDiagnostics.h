/*
 * @Author: ELEGOO (Enhanced by Senior Developer)
 * @Date: 2021-01-04 (Enhanced: 2025-01-25)
 * @Description: System Diagnostics and Health Monitoring
 * @FilePath: SystemDiagnostics.h
 */

#ifndef _SystemDiagnostics_H_
#define _SystemDiagnostics_H_

#include <arduino.h>

// Diagnostic levels
#define DIAG_LEVEL_INFO    0
#define DIAG_LEVEL_WARNING 1
#define DIAG_LEVEL_ERROR   2
#define DIAG_LEVEL_CRITICAL 3

// System health status codes
#define HEALTH_OK           0
#define HEALTH_WARNING      1
#define HEALTH_ERROR        2
#define HEALTH_CRITICAL     3

class SystemDiagnostics
{
public:
  void init();
  void logMessage(uint8_t level, const char* component, const char* message);
  void logSensorError(const char* sensor, uint16_t errorCode);
  uint8_t getSystemHealth();
  void printSystemStatus();
  void resetErrorCounters();

  // Sensor validation functions
  bool validateUltrasonicReading(uint16_t distance);
  bool validateVoltageReading(float voltage);
  bool validateServoPosition(uint8_t position);

  // Performance monitoring
  void recordOperationTime(const char* operation, unsigned long duration);
  void printPerformanceStats();

private:
  uint16_t errorCounts[4]; // Info, Warning, Error, Critical
  unsigned long lastDiagnosticTime;
  uint8_t currentSystemHealth;

  // Performance tracking
  struct OperationStats {
    unsigned long totalTime;
    uint16_t callCount;
    unsigned long maxTime;
  };

  OperationStats sensorReadStats;
  OperationStats motionControlStats;
  OperationStats serialParseStats;

  void updateSystemHealth();
  const char* getLevelString(uint8_t level);
};

extern SystemDiagnostics diagnostics;

#endif