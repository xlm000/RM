# SEGGER模块

## 概述
SEGGER RTT搭配SystemView使用，实时记录和可视化工具。它可以将数据从设备发送到 PC，并显示在 PC 上的图形化界面中。

## 用户配置（要求手动操作，用前必看）
1. 以stm32h723xx为例配置，如需更换芯片请在dev_segger.c中更换对应的.h文件
2. 在FreeRTOSConfig.h中添加```#include "dev_segger.h"```  
示例
```c
/* USER CODE BEGIN Defines */
#include "dev_segger.h"
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */
```
3. 在.ld文件中修改存储位置，否则systemview会显示```Could not find systemview buffer```  
示例：
```c
/* Specify the memory areas */
MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K  <-修改成RAM_D1
RAM_D1 (xrw)       : ORIGIN = 0x24000000, LENGTH = 320K  
RAM_D2 (xrw)       : ORIGIN = 0x30000000, LENGTH = 32K
RAM_D3 (xrw)       : ORIGIN = 0x38000000, LENGTH = 16K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx)         : ORIGIN = 0x8000000, LENGTH = 1024K
}
```
4. 设置systemview用于识别和显示内存地址和最低RAM基地址（要求和存储位置保持一致），一共修改两个文件```SEGGER_SYSVIEW_Config_FreeRTOS.c```
```c
// The application name to be displayed in SystemViewer
#define SYSVIEW_APP_NAME        "<project_name>"  <-按需求修改项目名称

// The target device name
#define SYSVIEW_DEVICE_NAME     "Cortex-M7"

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_GET_TIMESTAMP in SEGGER_SYSVIEW_Conf.h
#define SYSVIEW_TIMESTAMP_FREQ  (configCPU_CLOCK_HZ)

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#define SYSVIEW_CPU_FREQ        configCPU_CLOCK_HZ

// The lowest RAM address used for IDs (pointers)
#define SYSVIEW_RAM_BASE        (0x24000000)        <-修改地址
```
```SEGGER_SYSVIEW_Config.h```
```c
#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE      4096
#define SEGGER_SYSVIEW_ID_BASE              0x24000000<-修改地址
#define SEGGER_SYSVIEW_ID_SHIFT             2
```
## 注意事项
1. 中断名称注册函数：SystemView_Register_ISRs()用于注册所有中断名称
2. FreeRTOS集成：FreeRTOSConfig.h包含SystemView 头文件。当 FreeRTOS 发生中断时，会自动调用 SystemView 的中断钩子函数来记录中断事件，实现在 SystemView 工具中的中断跟踪和分析功能。
3. 中断钩子函数
```c
  vApplicationMallocFailedHook();                   //内存分配失败钩子
  vApplicationStackOverflowHook(xTask,pcTaskName);  //任务栈溢出钩子
  vApplicationIdleHook();                           //系统空闲任务钩子
  vMainConfigureTimerForRunTimeStats();             //运行时统计定时器配置
  ulMainGetRunTimeCounterValue();                   //获取运行时计数值
  vApplicationTickHook();                           //系统节拍钩子，在每个系统tick中调用
```
## 参考例程
```c
#ifdef USE_SYSVIEW
SEGGER_SystemView_Init();
#endif
```