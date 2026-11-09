//
// Created by 朱泽超 on 2026/11/4.
//

#include "ChassisTask.h"
#include "cmsis_os.h"
#include "dev_dr16.h"

//底盘config
ChassisInitConfig_s chassis_config = {
    .type = Omni_Wheel,
    .motor_config[0] = {
        .topic_name = "left_front_wheel",
        .type = CHASSIS_WHEELMOTOR_TYPE,
        .control_mode = CHASSIS_WHEELMOTOR_MODE,
        .id = 1,
        .can_config = {
            .can_number = CHASSIS_WHEELMOTOR_CANNUMBER,
            .rx_id = 0x205,
            .tx_id = 0x1FF
        },
        .reduction_ratio = 1.0f,
        .velocity_pid_config = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .angle_max = 0.0f,
            .i_max = 0.0f,
            .out_max = 0.0f,
        },
    },
    .motor_config[1] = {
        .topic_name = "right_front_wheel",
        .type = CHASSIS_WHEELMOTOR_TYPE,
        .control_mode = CHASSIS_WHEELMOTOR_MODE,
        .id = 2,
        .can_config = {
            .can_number = CHASSIS_WHEELMOTOR_CANNUMBER,
            .rx_id = 0x206,
            .tx_id = 0x1FF
        },
        .reduction_ratio = 1.0f,
        .velocity_pid_config = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .angle_max = 0.0f,
            .i_max = 0.0f,
            .out_max = 0.0f,
        },
    },
    .motor_config[2] = {
        .topic_name = "right_back_wheel",
        .type = CHASSIS_WHEELMOTOR_TYPE,
        .control_mode = CHASSIS_WHEELMOTOR_MODE,
        .id = 3,
        .can_config = {
            .can_number = CHASSIS_WHEELMOTOR_CANNUMBER,
            .rx_id = 0x207,
            .tx_id = 0x1FF
        },
        .reduction_ratio = 1.0f,
        .velocity_pid_config = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .angle_max = 0.0f,
            .i_max = 0.0f,
            .out_max = 0.0f,
        },
    },
    .motor_config[3] = {
        .topic_name = "left_back_wheel",
        .type = CHASSIS_WHEELMOTOR_TYPE,
        .control_mode = CHASSIS_WHEELMOTOR_MODE,
        .id = 4,
        .can_config = {
            .can_number = CHASSIS_WHEELMOTOR_CANNUMBER,
            .rx_id = 0x208,
            .tx_id = 0x1FF
        },
        .reduction_ratio = 1.0f,
        .velocity_pid_config = {
            .kp = 0.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .angle_max = 0.0f,
            .i_max = 0.0f,
            .out_max = 0.0f,
        },
    },
    .gimbal_follow_pid_config = {
        .kp = 0.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .angle_max = 0.0f,
        .i_max = 0.0f,
        .out_max = 0.0f,
    },
    .omni_steering_message ={
        .wheel_radius = 0.0765f,
        .chassis_radius = 0.27830149f,
    },
    .gimbal_yaw_zero = 0.0f,
    .Gyroscope_Speed = 0.0f,
};
//实例注册区
ChassisInstance_s *chassis_instance;
Dr16Instance_s *dr16_instance;
//特殊变量赋值区
chassis_state_e chassis_state;

/**
 * @brief 遥控器控制底盘状态函数
 * @param dr16_instance
 * @return 底盘状态chassis_state_e
 * @note 中中独立，中下跟随，下中陀螺，其他失能
 */
static chassis_state_e Chassis_State(Dr16Instance_s *dr16_instance)
{
    if (dr16_instance->dr16_handle.s1 == 1 || dr16_instance == NULL)
    {
        return CHASSIS_DISABLE;
    }
    else if (dr16_instance->dr16_handle.s1 == 3)
    {
        if (dr16_instance->dr16_handle.s2 == 3)
            return CHASSIS_ENABLE_NORMAL;
        if (dr16_instance->dr16_handle.s2 == 2)
            return CHASSIS_ENABLE_FOLLOW_GIMBAL;
    }
    else if (dr16_instance->dr16_handle.s1 == 2)
    {
        if (dr16_instance->dr16_handle.s2 == 1)
            return CHASSIS_ENABLE_GYROSCOPE;
    }
    else
    {
        return CHASSIS_DISABLE;
    }

}

//底盘freertos函数
void ChassisTask(void const * argument)
{
    //注册函数区
    chassis_instance = Chassis_Register(&chassis_config);
    dr16_instance = Dr16_Register(&huart6);

    for (;;)
    {
        //
        chassis_state = Chassis_State(dr16_instance);
        switch (chassis_state)
        {
        case CHASSIS_DISABLE: //失能
            Motor_Dji_SetCurrent(chassis_instance->chassis_motor[0], 0);
            Motor_Dji_SetCurrent(chassis_instance->chassis_motor[1], 0);
            Motor_Dji_SetCurrent(chassis_instance->chassis_motor[2], 0);
            Motor_Dji_SetCurrent(chassis_instance->chassis_motor[3], 0);
            Chassis_Transmit(chassis_instance);
            break;
        case CHASSIS_ENABLE_NORMAL: //独立运动
            Chassis_Change_Mode(chassis_instance, CHASSIS_NORMAL);
            Chassis_Control(chassis_instance);
            break;
        case CHASSIS_ENABLE_FOLLOW_GIMBAL: //跟随云台
            Chassis_Change_Mode(chassis_instance, CHASSIS_FOLLOW_GIMBAL);
            Chassis_Control(chassis_instance);
            break;
        case CHASSIS_ENABLE_GYROSCOPE: //小陀螺
            Chassis_Change_Mode(chassis_instance, CHASSIS_GYROSCOPE);
            Chassis_Control(chassis_instance);
            break;
        }
    }
}