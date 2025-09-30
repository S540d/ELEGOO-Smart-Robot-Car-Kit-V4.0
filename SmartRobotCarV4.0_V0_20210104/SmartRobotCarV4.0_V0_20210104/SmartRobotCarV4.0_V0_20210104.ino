/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2024-09-30 (ESP32 Brain Architecture)
 * @LastEditors: Changhua + ESP32 Brain Integration
 * @Description: Smart Robot Car V4.0 - ESP32 Brain Architecture
 * @FilePath:
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

// ESP32 Brain Architecture - Optional ESP32 communication
// If no ESP32 connected, all existing control methods still work!
#define ENABLE_ESP32_BRAIN 1

#if ENABLE_ESP32_BRAIN
#include "ESP32CommunicationHandler.h"
ESP32CommunicationHandler esp32Handler;
#endif

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);

#if ENABLE_ESP32_BRAIN
  esp32Handler.init();
#endif
}

void loop()
{
  //put your main code here, to run repeatedly :
  wdt_reset();

#if ENABLE_ESP32_BRAIN
  // ESP32 Brain: Process commands and send sensor data
  esp32Handler.processCommands();
  esp32Handler.sendSensorData();
#endif

  // Standard sensor updates and control methods
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_Follow();
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  Application_FunctionSet.ApplicationFunctionSet_Rocker();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  Application_FunctionSet.ApplicationFunctionSet_IRrecv();

#if !ENABLE_ESP32_BRAIN
  // Only process standard serial if ESP32 brain disabled
  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();
#endif

  // Standard command processing
  Application_FunctionSet.CMD_ServoControl_xxx0();
  Application_FunctionSet.CMD_MotorControl_xxx0();
  Application_FunctionSet.CMD_CarControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_CarControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_MotorControlSpeed_xxx0();
  Application_FunctionSet.CMD_LightingControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_LightingControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_ClearAllFunctions_xxx0();
}
