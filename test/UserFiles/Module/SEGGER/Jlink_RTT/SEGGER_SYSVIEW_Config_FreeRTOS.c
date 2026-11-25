/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2021 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: 3.30                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_FreeRTOS.c
Purpose : Sample setup configuration of SystemView with FreeRTOS.
Revision: $Rev: 7745 $
*/
#include "FreeRTOS.h"
#include "SEGGER_SYSVIEW.h"

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "9Axis_Arm_MC02_H7"

// The target device name
#define SYSVIEW_DEVICE_NAME     "Cortex-M7"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (configCPU_CLOCK_HZ)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        configCPU_CLOCK_HZ

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x24000000)

/********************************************************************* 
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME",D="SYSVIEW_DEVICE_NAME",O=FreeRTOS");
  SEGGER_SYSVIEW_SendSysDesc("I#15=SysTick");

  // ========= 所有的中断名称映射 =========
  // SystemView 的语法规范是 "I#<Exception号>=<中断名>"
  // Exception号 = 硬件的 IRQ号 + 16
  SEGGER_SYSVIEW_SendSysDesc("I#27=DMA1_Stream0"); // IRQ 11
  SEGGER_SYSVIEW_SendSysDesc("I#28=DMA1_Stream1"); // IRQ 12
  SEGGER_SYSVIEW_SendSysDesc("I#29=DMA1_Stream2"); // IRQ 13
  SEGGER_SYSVIEW_SendSysDesc("I#30=DMA1_Stream3"); // IRQ 14
  SEGGER_SYSVIEW_SendSysDesc("I#31=DMA1_Stream4"); // IRQ 15
  SEGGER_SYSVIEW_SendSysDesc("I#32=DMA1_Stream5"); // IRQ 16
  SEGGER_SYSVIEW_SendSysDesc("I#33=DMA1_Stream6"); // IRQ 17
  SEGGER_SYSVIEW_SendSysDesc("I#35=FDCAN1_IT0");   // IRQ 19
  SEGGER_SYSVIEW_SendSysDesc("I#36=FDCAN2_IT0");   // IRQ 20
  SEGGER_SYSVIEW_SendSysDesc("I#44=TIM2");         // IRQ 28
  SEGGER_SYSVIEW_SendSysDesc("I#52=SPI2");         // IRQ 36
  SEGGER_SYSVIEW_SendSysDesc("I#53=USART1");       // IRQ 37
  SEGGER_SYSVIEW_SendSysDesc("I#54=USART2");       // IRQ 38
  SEGGER_SYSVIEW_SendSysDesc("I#55=USART3");       // IRQ 39
  SEGGER_SYSVIEW_SendSysDesc("I#63=DMA1_Stream7"); // IRQ 47
  SEGGER_SYSVIEW_SendSysDesc("I#69=UART5");        // IRQ 53
  SEGGER_SYSVIEW_SendSysDesc("I#72=DMA2_Stream0"); // IRQ 56
  SEGGER_SYSVIEW_SendSysDesc("I#73=DMA2_Stream1"); // IRQ 57
  SEGGER_SYSVIEW_SendSysDesc("I#74=DMA2_Stream2"); // IRQ 58
  SEGGER_SYSVIEW_SendSysDesc("I#75=DMA2_Stream3"); // IRQ 59
  SEGGER_SYSVIEW_SendSysDesc("I#76=DMA2_Stream4"); // IRQ 60
  SEGGER_SYSVIEW_SendSysDesc("I#85=DMA2_Stream6"); // IRQ 69
  SEGGER_SYSVIEW_SendSysDesc("I#86=DMA2_Stream7"); // IRQ 70
  SEGGER_SYSVIEW_SendSysDesc("I#93=OTG_HS");       // IRQ 77
  SEGGER_SYSVIEW_SendSysDesc("I#98=UART7");        // IRQ 82
  SEGGER_SYSVIEW_SendSysDesc("I#172=USART10");     // IRQ 156
  SEGGER_SYSVIEW_SendSysDesc("I#175=FDCAN3_IT0");  // IRQ 159
}

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ, 
                      &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

/*************************** End of file ****************************/
