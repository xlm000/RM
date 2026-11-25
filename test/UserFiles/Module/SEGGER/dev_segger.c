#include "dev_segger.h"
#include "SEGGER_SYSVIEW.h"

#ifdef USE_SYSVIEW
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "stm32h723xx.h"


/*********************************************************************
*
*       vApplicationMallocFailedHook
*
*  Function description
*    Called if a call to pvPortMalloc() fails because there
*    is insufficient free memory available in the FreeRTOS heap.
*    pvPortMalloc() is called internally by FreeRTOS API functions
*    that create tasks, queues, software timers, and semaphores.
*    The size of the FreeRTOS heap is set by the configTOTAL_HEAP_SIZE
*    configuration constant in FreeRTOSConfig.h
*
*/
void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*********************************************************************
*
*       vApplicationStackOverflowHook
*
*  Function description
*    Run time stack overflow checking is performed if
*    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.
*    This hook function is called if a stack overflow is detected.
*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName) {
  ( void ) pcTaskName;
  ( void ) xTask;
  taskDISABLE_INTERRUPTS();
  for( ;; );
}
/*********************************************************************
*
*       vApplicationIdleHook
*
*  Function description
*    This function is called on each cycle of the idle task.
*    In this case it does nothing useful, other than report
*    the amount of FreeRTOS heap that remains unallocated.
*
*/
void vApplicationIdleHook(void) {
#if configSUPPORT_DYNAMIC_ALLOCATION == 1
  volatile size_t xFreeHeapSpace;

  xFreeHeapSpace = xPortGetFreeHeapSize();
  if( xFreeHeapSpace > 100 ) {
    //
    // By now, the kernel has allocated everything it is going to, so
    // if there is a lot of heap remaining unallocated then
    // the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
    // reduced accordingly.
  }
#endif
}
/*********************************************************************
*
*       vMainConfigureTimerForRunTimeStats
*
*  Function description
*    The Blinky build configuration does not include run time
*    stats gathering, however, the Full and Blinky build configurations
*    share a FreeRTOSConfig.h file.
*    Therefore, dummy run time stats functions need to be defined
*    to keep the linker happy.
*
*/
void vMainConfigureTimerForRunTimeStats(void) {
}
/*********************************************************************
*
*       ulMainGetRunTimeCounterValue
*
*/
unsigned long ulMainGetRunTimeCounterValue(void) {
  return 0UL;
}
/*********************************************************************
*
*       vApplicationTickHook
*
*  Function description
*    A tick hook is used by the "Full" build configuration.
*    The Full and blinky build configurations share a FreeRTOSConfig.h
*    header file, so this simple build configuration also has to define
*    a tick hook - even though it does not actually use it for anything.
*
*/
void vApplicationTickHook(void) {
}

void SEGGER_SystemView_Init(void){
  SEGGER_SYSVIEW_Conf();
  SystemView_Register_ISRs();
}

/**
 * @brief 自动向 SystemView 注册中断名称
 * @note 必须在 SEGGER_SYSVIEW_Start() 之后调用
 */
void SystemView_Register_ISRs(void)
{
    char desc_buf[48];

    // 使用系统自带的中断宏 (IRQn)，自动加上16偏移装换成 SystemView 识别的 Exception Number
    #define REG_ISR(irqn, name) \
    snprintf(desc_buf, sizeof(desc_buf), "I#%d=%s", (int)(irqn) + 16, name); \
    SEGGER_SYSVIEW_SendSysDesc(desc_buf)

    // ======= 根据你的 stm32h7xx_it.c 提取的全部中断 =======

    // DMA1 (主要用于 SPI, UART, USART_RX)
    REG_ISR(DMA1_Stream0_IRQn, "DMA1_S0_ADC3");
    REG_ISR(DMA1_Stream1_IRQn, "DMA1_S1_SPI2RX");
    REG_ISR(DMA1_Stream2_IRQn, "DMA1_S2_SPI2TX");
    REG_ISR(DMA1_Stream3_IRQn, "DMA1_S3_UART7RX");
    REG_ISR(DMA1_Stream4_IRQn, "DMA1_S4_UART7TX");
    REG_ISR(DMA1_Stream5_IRQn, "DMA1_S5_USART1RX");
    REG_ISR(DMA1_Stream6_IRQn, "DMA1_S6_USART1TX");
    REG_ISR(DMA1_Stream7_IRQn, "DMA1_S7_USART2RX");

    // DMA2 (主要用于 USART_TX, USART10, ADC1, M2M)
    REG_ISR(DMA2_Stream0_IRQn, "DMA2_S0_USART2TX");
    REG_ISR(DMA2_Stream1_IRQn, "DMA2_S1_USART3RX");
    REG_ISR(DMA2_Stream2_IRQn, "DMA2_S2_USART3TX");
    REG_ISR(DMA2_Stream3_IRQn, "DMA2_S3_USART10RX");
    REG_ISR(DMA2_Stream4_IRQn, "DMA2_S4_USART10TX");
    REG_ISR(DMA2_Stream6_IRQn, "DMA2_S6_ADC1");
    REG_ISR(DMA2_Stream7_IRQn, "DMA2_S7_MEM2MEM");

    // CAN 总线
    REG_ISR(FDCAN1_IT0_IRQn, "FDCAN1_IT0");
    REG_ISR(FDCAN2_IT0_IRQn, "FDCAN2_IT0");
    REG_ISR(FDCAN3_IT0_IRQn, "FDCAN3_IT0");

    // 定时器与外设
    REG_ISR(TIM2_IRQn,       "TIM2");
    REG_ISR(SPI2_IRQn,       "SPI2");
    REG_ISR(OTG_HS_IRQn,     "USB_OTG_HS");

    // 串口
    REG_ISR(USART1_IRQn,     "USART1");
    REG_ISR(USART2_IRQn,     "USART2");
    REG_ISR(USART3_IRQn,     "USART3");
    REG_ISR(UART5_IRQn,      "UART5");
    REG_ISR(UART7_IRQn,      "UART7");
    REG_ISR(USART10_IRQn,    "USART10");
}
#endif

