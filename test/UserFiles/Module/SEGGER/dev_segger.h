/**
 * @file dev_segger.h
 * @brief SEGGER SystemView 调试追踪模块
 */
#ifndef DEV_SEGGER_H
#define DEV_SEGGER_H
#include "robot_config.h"
#include "SEGGER_RTT.h"
//#include "SEGGER_SYSVIEW_FreeRTOS.h"
#ifdef USE_SYSVIEW
void SEGGER_SystemView_Init(void);
void SystemView_Register_ISRs(void);
#endif
#endif //DEV_SEGGER_H