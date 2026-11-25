# dev_motor_dm

## 概述
dev_motor_dm是一个用于驱动达妙电机的程序，提供了达妙电机的控制功能。

## 开发环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL

## 适配电机
- J4310
- J4340
- S3519

## 软件要求
- 需要bsp_can, alg_pid, bsp_log库

## 注意事项
1. 使用前请确保已正确配置CAN总线。
2. 所有配置请更具实际需求进行调整,可通过上位机查看
3. 使用前请仔细阅读达妙电机的相关文档和手册
4. 确保CAN总线配置正确，波特率等参数与电机匹配。
5. 实例中的`cnt`是统计通信次数的，但是超过65535时会自动归零，如果需要使用请在app层自行清零。
6. DM电机的`total_angle`是累计输出轴角度值，这里设置是超过`FLOAT_MAX`会**清零**，如果需要使用请需要在app层自行处理。
    提示：可以在读取`total_angle`大于某一个值的时候进行周期性清零。


## 示例
```c
#include "dev_motor_dm.h"

DmMotorInstance_s *dm_motor = NULL;

DmMotorInitConfig_s dm_config={
    .can_config = {
        .can_number = 1,
        .tx_id = 0x200,
        .rx_id = 0x1FF,
    },
    .control_mode = DM_VELOCITY,
    .parameters = { // 这几个数据是我瞎编的，不具有任何参考性
        .pos_max = 3.14159f,
        .vel_max = 20.0f,   
        .tor_max = 18.0f,
        .kp_max = 100.0f,
        .kd_max = 10.0f,
        .kp_int = 2.0f,
        .kd_int = 1.0f,
    },
    .control_mode = DM_VELOCITY,
    .topic_name = "dm_motor_test",
    .type = J4310,
    .velocity_pid_config = {
        .kp = 20.0f,
        .ki = 0.5f,
        .kd = 0.0f,
        .i_max = 10.0f,
        .out_max = 100.0f,
    },
};

void test_task(void *pvParameters){
    dm_motor = Motor_DM_Register(&dm_config);
    Motor_Dm_Cmd(dm_motor, DM_CMD_MOTOR_ENABLE);
    Motor_Dm_Transmit(dm_motor);
    // 这里最好加一个是否注册成功的判断，示例中不做演示
    for( ; ; ){
        Motor_DM_Control(dm_motor, 10.0f); // 计算力矩
        Motor_Dm_Mit_Control(dm_motor, 0.0f, 0.0f, dm_motor->output); // 电机为MIT模式
        Motor_Dm_Transmit(dm_motor); //发送控制报文
        osDelay(1);
    }
} 
```
