# dev_motor_dji的
## 概述
dev_motor_dji是一个用于驱动大疆电机的程序，提供了大疆电机的控制功能。

## 开发环境
- 硬件平台：STM32F4系列(RoboMasterC板)
- 开发工具：Keil MDK-ARM
- 相关库：CMSIS, HAL

## 适配电机
- 大疆M2006、M3508、GM6020

## 软件要求
- 需要bsp_can、alg_pid库
- 需要正确配置CAN总线

## 注意事项：
1. 确保CAN总线配置正确，波特率等参数与电机匹配。
2. 实例中的`cnt`是统计通信次数的，但是超过65535时会自动归零，如果需要使用请在app层自行清零。
3. DJI电机的`total_angle`是累计输出轴角度值，这里设置是超过`FLOAT_MAX`会**清零**，如果需要使用请需要在app层自行处理。
    提示：可以在读取`total_angle`大于某一个值的时候进行周期性清零。
4. 如果使用`Motor_Dji_Control`，控制的是电机输出轴,而非转子角度
5. 电机信息如减速箱设置等必须与实际一致，否则会导致控制不准确

## 实例
```c
#include "dev_motor_dji.h"
DjiMotorInstance_s *motor;
DjiMotorInitConfig_s config = {
    .reduction_ratio = 19f,
    .control_mode = DJI_VELOCITY,
    .type = M2006,
    .topic_name = "dji_motor",
    .velocity_pid_config = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_output = 10000.0f,
    .can_config = {
        .can_number = 1,
        .tx_id = 0x200,
        .rx_id = 0x201,
    }
    }

    void test_task(void) {
        motor = Motor_Dji_Register(&config);
        while (1) {
            Motor_Dji_Control(motor, 1000); // 以1000rpm的速度运行电机
            Motor_Dji_Transmit(motor); // 发送控制命令
            osDelay(1);
        }
    }
};



