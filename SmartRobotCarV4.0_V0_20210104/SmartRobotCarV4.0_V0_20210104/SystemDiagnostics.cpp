/*
 * @Author: ELEGOO (Enhanced by Senior Developer)
 * @Date: 2021-01-04 (Enhanced: 2025-01-25)
 * @Description: System Diagnostics and Health Monitoring Implementation
 * @FilePath: SystemDiagnostics.cpp
 */

#include "SystemDiagnostics.h"
#include "DeviceDriverSet_xxx0.h"

SystemDiagnostics diagnostics;

void SystemDiagnostics::init()
{
  for (int i = 0; i < 4; i++) {
    errorCounts[i] = 0;
  }

  sensorReadStats = {0, 0, 0};
  motionControlStats = {0, 0, 0};
  serialParseStats = {0, 0, 0};

  currentSystemHealth = HEALTH_OK;
  lastDiagnosticTime = _millis();

  logMessage(DIAG_LEVEL_INFO, "SYSTEM", "Diagnostics initialized");
}

void SystemDiagnostics::logMessage(uint8_t level, const char* component, const char* message)
{
  errorCounts[level]++;

  Serial.print("[");
  Serial.print(getLevelString(level));
  Serial.print("] ");
  Serial.print(component);
  Serial.print(": ");
  Serial.println(message);

  updateSystemHealth();
}

void SystemDiagnostics::logSensorError(const char* sensor, uint16_t errorCode)
{
  Serial.print("[ERROR] SENSOR ");
  Serial.print(sensor);
  Serial.print(" Error Code: ");
  Serial.println(errorCode);

  errorCounts[DIAG_LEVEL_ERROR]++;
  updateSystemHealth();
}

uint8_t SystemDiagnostics::getSystemHealth()
{
  return currentSystemHealth;
}

void SystemDiagnostics::printSystemStatus()
{
  Serial.println("=== SYSTEM STATUS ===");
  Serial.print("Health: ");

  switch(currentSystemHealth) {
    case HEALTH_OK:      Serial.println("OK"); break;
    case HEALTH_WARNING: Serial.println("WARNING"); break;
    case HEALTH_ERROR:   Serial.println("ERROR"); break;
    case HEALTH_CRITICAL: Serial.println("CRITICAL"); break;
  }

  Serial.print("Uptime: ");
  Serial.print(_millis() / 1000);
  Serial.println(" seconds");

  Serial.println("Error Counts:");
  Serial.print("  Info: "); Serial.println(errorCounts[0]);
  Serial.print("  Warning: "); Serial.println(errorCounts[1]);
  Serial.print("  Error: "); Serial.println(errorCounts[2]);
  Serial.print("  Critical: "); Serial.println(errorCounts[3]);

  Serial.println("====================");
}

void SystemDiagnostics::resetErrorCounters()
{
  for (int i = 0; i < 4; i++) {
    errorCounts[i] = 0;
  }
  currentSystemHealth = HEALTH_OK;
  logMessage(DIAG_LEVEL_INFO, "SYSTEM", "Error counters reset");
}

bool SystemDiagnostics::validateUltrasonicReading(uint16_t distance)
{
  if (distance < ULTRASONIC_MIN_DISTANCE || distance > ULTRASONIC_MAX_DISTANCE) {
    logSensorError("ULTRASONIC", distance);
    return false;
  }
  return true;
}

bool SystemDiagnostics::validateVoltageReading(float voltage)
{
  if (voltage < 5.0 || voltage > 12.0) {
    logSensorError("VOLTAGE", (uint16_t)(voltage * 100));
    return false;
  }

  if (voltage < BATTERY_LOW_THRESHOLD) {
    logMessage(DIAG_LEVEL_WARNING, "POWER", "Low battery voltage detected");
    return false;
  }

  return true;
}

bool SystemDiagnostics::validateServoPosition(uint8_t position)
{
  if (position > 180) {
    logSensorError("SERVO", position);
    return false;
  }
  return true;
}

void SystemDiagnostics::recordOperationTime(const char* operation, unsigned long duration)
{
  OperationStats* stats = nullptr;

  if (strcmp(operation, "sensor") == 0) {
    stats = &sensorReadStats;
  } else if (strcmp(operation, "motion") == 0) {
    stats = &motionControlStats;
  } else if (strcmp(operation, "serial") == 0) {
    stats = &serialParseStats;
  }

  if (stats) {
    stats->totalTime += duration;
    stats->callCount++;
    if (duration > stats->maxTime) {
      stats->maxTime = duration;
    }
  }
}

void SystemDiagnostics::printPerformanceStats()
{
  Serial.println("=== PERFORMANCE STATS ===");

  Serial.println("Sensor Operations:");
  Serial.print("  Avg: ");
  Serial.print(sensorReadStats.callCount > 0 ? sensorReadStats.totalTime / sensorReadStats.callCount : 0);
  Serial.print("ms, Max: "); Serial.print(sensorReadStats.maxTime);
  Serial.print("ms, Count: "); Serial.println(sensorReadStats.callCount);

  Serial.println("Motion Control:");
  Serial.print("  Avg: ");
  Serial.print(motionControlStats.callCount > 0 ? motionControlStats.totalTime / motionControlStats.callCount : 0);
  Serial.print("ms, Max: "); Serial.print(motionControlStats.maxTime);
  Serial.print("ms, Count: "); Serial.println(motionControlStats.callCount);

  Serial.println("Serial Parsing:");
  Serial.print("  Avg: ");
  Serial.print(serialParseStats.callCount > 0 ? serialParseStats.totalTime / serialParseStats.callCount : 0);
  Serial.print("ms, Max: "); Serial.print(serialParseStats.maxTime);
  Serial.print("ms, Count: "); Serial.println(serialParseStats.callCount);

  Serial.println("========================");
}

void SystemDiagnostics::updateSystemHealth()
{
  uint8_t newHealth = HEALTH_OK;

  if (errorCounts[DIAG_LEVEL_CRITICAL] > 0) {
    newHealth = HEALTH_CRITICAL;
  } else if (errorCounts[DIAG_LEVEL_ERROR] > 5) {
    newHealth = HEALTH_ERROR;
  } else if (errorCounts[DIAG_LEVEL_WARNING] > 10) {
    newHealth = HEALTH_WARNING;
  }

  if (newHealth != currentSystemHealth) {
    currentSystemHealth = newHealth;
    Serial.print("System health changed to: ");
    Serial.println(getLevelString(newHealth));
  }
}

const char* SystemDiagnostics::getLevelString(uint8_t level)
{
  switch(level) {
    case DIAG_LEVEL_INFO:     return "INFO";
    case DIAG_LEVEL_WARNING:  return "WARN";
    case DIAG_LEVEL_ERROR:    return "ERROR";
    case DIAG_LEVEL_CRITICAL: return "CRIT";
    default:                  return "UNKNOWN";
  }
}