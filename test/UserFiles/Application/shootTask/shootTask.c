//
// Created by 朱泽超 on 2026/8/3.
//

#include "shootTask.h"
#include "alg_pid.h"
#include "alg_fliter.h"
#include "bsp_can.h"
#include "dev_et08a.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#include "dev_referee.h"
#include "usart.h"

#include "cmsis_os.h"

#include <math.h>

//裁判系统注册
RefereeInitConfig_s ref_config = {
    .topic_name = "referee",
    .uart_handle = &huart6,
    .mode = UART_IDLE_MODE
};
//电机注册
DjiMotorInitConfig_s wheel_1 = {
    .topic_name = "wheel_1",
    .type = M2006,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .id = 1,
    .can_config = {
        .can_number = 1,
        .rx_id = 0x201,
        .tx_id = 0x200
    },
    .velocity_pid_config = {
        .i_max = 6000.0f,
        .out_max = 10000.0f,
        .kp = 8.0f,
        .ki = 0.001f,
        .kd = 6.0f,
    }
};
DjiMotorInitConfig_s wheel_2 = {
    .topic_name = "wheel_2",
    .type = M2006,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .id = 3,
    .can_config = {
        .can_number = 1,
        .rx_id = 0x203,
        .tx_id = 0x200
    },
    .velocity_pid_config = {
        .i_max = 6000.0f,
        .out_max = 10000.0f,
        .kp = 7.0f,
        .ki = 0.001f,
        .kd = 5.0f,
    }
};
DjiMotorInitConfig_s wheel_3 = {
    .topic_name = "wheel_3",
    .type = M2006,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 1.0f,
    .id = 6,
    .can_config = {
        .can_number = 1,
        .rx_id = 0x206,
        .tx_id = 0x1FF
    },
    .velocity_pid_config = {
        .i_max = 6000.0f,
        .out_max = 10000.0f,
        .kp = 7.0f,
        .ki = 0.001f,
        .kd = 5.0f,
    }
};
DjiMotorInitConfig_s dial = {
    .topic_name = "dial",
    .type = M2006,
    .control_mode = DJI_POSITION,
    .reduction_ratio = 36.0f,
    .id = 5,
    .can_config = {
        .can_number = 1,
        .rx_id = 0x205,
        .tx_id = 0x1FF
    },
    .velocity_pid_config = {
        .i_max = 4000.0f,
        .out_max = 6000.0f,
        .kp = 450.0f,
        .ki = 1.0f,
        .kd = 0.0f,
    },
    .angle_pid_config = {
        .i_max = 0.0f,
        .out_max = 300.0f,
        .kp = 100.0f,
        .ki = 0.0f,
        .kd = 0.0f,
    }
};
DmMotorInitConfig_s pitch = {
    .topic_name = "pitch",
    .type = J4310,
    .control_mode = DM_POSITION,
    .parameters = {
        .pos_max = 3.1415926f,
        .tor_max = 7.0f,
        .vel_max = 40.0f,
    },
    .can_config = {
        .can_number = 1,
        .rx_id = 0x19,
        .tx_id = 0x09
    },
    .velocity_pid_config = {
        .i_max = 3.0f,
        .out_max = 7.0f,
        .kp = 0.6f,
        .ki = 0.015f,
        .kd = 0.0f,
    },
    .angle_pid_config = {
        .i_max = 0.0f,
        .out_max = 15.0f,
        .kp = 15.0f,
        .ki = 0.0f,
        .kd = 6.0f,
        .angle_max = 2*3.1415926f
    }
};
//低通过滤器注册
FilterInitConfig_t Lowpass_wheel_config = {
    .cutoff_freq = 15000.0f,
    .sample_freq = 100000.0f
};

//电机实例
DjiMotorInstance_s *wheel_1_instance;
DjiMotorInstance_s *wheel_2_instance;
DjiMotorInstance_s *wheel_3_instance;
DjiMotorInstance_s *dial_instance;
DmMotorInstance_s *pitch_instance;
//遥控器实例
Et08aInstance_s *Et08a;
//裁判系统实例
RefereeInstance_s *ref_instance;
//低通滤波器实例
LowpassFilter_t *Lowpass_wheel_instance;

//初始常量设置
float wheel_velocity = 2000.0f;
float galaxy_offset = 0.0f; //pitch轴重力补偿
float fraction_offset = 0.0f; //pitch轴摩擦力补偿

//变量设置
float dial_target_angle = 0.0f;
float pitch_target_angle = 0.0f;

//测试用暂存变量
float wheel_1_velocity = 0.0f;
float wheel_2_velocity = 0.0f;
float wheel_3_velocity = 0.0f;


void shootTask()
{
    //裁判系统注册
    ref_instance = Referee_Register(&ref_config);
    //Et08a注册
    Et08a = Et08a_Register(&huart3);
    //低通滤波器注册
    Lowpass_wheel_instance = LowpassFilter_Register(&Lowpass_wheel_config);
    //电机注册
    wheel_1_instance = Motor_Dji_Register(&wheel_1);
    wheel_2_instance = Motor_Dji_Register(&wheel_2);
    wheel_3_instance = Motor_Dji_Register(&wheel_3);
    dial_instance = Motor_Dji_Register(&dial);
    pitch_instance = Motor_DM_Register(&pitch);

    uint8_t motor_check = 0;//单发检测
    uint16_t count=0;//连发延迟

    //目标位置初始化
    dial_target_angle = dial_instance->message.out_position;
    pitch_target_angle = pitch_instance->message.out_position;

    for (;;)
    {
        wheel_1_velocity = wheel_1_instance->message.out_velocity;
        wheel_2_velocity = wheel_2_instance->message.out_velocity;
        wheel_3_velocity = wheel_3_instance->message.out_velocity;

        //摩擦轮数据滤波
        if (wheel_1_instance != NULL && wheel_2_instance != NULL && wheel_3_instance != NULL)
        {
            //LowpassFilter_Process(Lowpass_wheel_instance, wheel_1_instance->message.out_velocity, &wheel_1_instance->message.out_velocity);
            //LowpassFilter_Process(Lowpass_wheel_instance, wheel_2_instance->message.out_velocity, &wheel_2_instance->message.out_velocity);
            //LowpassFilter_Process(Lowpass_wheel_instance, wheel_3_instance->message.out_velocity, &wheel_3_instance->message.out_velocity);
        }

        if (Et08a->sb == 1 || Et08a == NULL)
        {
            if (fabsf(wheel_1_velocity) >= 1200.0f)
            {
                //Motor_Dji_Control(wheel_1_instance, 0.0f);
            }
            else if (fabsf(wheel_1_velocity) < 1200.0f)
            {
                //Motor_Dji_SetCurrent(wheel_1_instance, 0.0001f);
            }
            if (fabsf(wheel_2_velocity) >= 1200.0f)
            {
                //Motor_Dji_Control(wheel_2_instance, 0.0f);
            }
            else if (fabsf(wheel_2_velocity) < 1200.0f)
            {
                //Motor_Dji_SetCurrent(wheel_2_instance, 0.0001f);
            }
            if (fabsf(wheel_3_velocity) >= 1200.0f)
            {
                //Motor_Dji_Control(wheel_3_instance, 0.0001f);
            }
            else if (fabsf(wheel_3_velocity) < 1200.0f)
            {
                //Motor_Dji_SetCurrent(wheel_3_instance, 0.0001f);
            }
        }

        if (Et08a->sb == 3) //单发
        {
            //Motor_Dji_Control(wheel_1_instance, -wheel_velocity);
            //Motor_Dji_Control(wheel_2_instance, -wheel_velocity);
            //Motor_Dji_Control(wheel_3_instance, -wheel_velocity);

            if (Et08a->sa == 1 && motor_check == 0)
            {
                //if ((dial_target_angle - dial_instance->message.out_position) < M_PI/2.0f)
                    dial_target_angle += 44.98f/180.0f*M_PI;
                motor_check = 1;
            }
            if (Et08a->sa == 2 && motor_check == 1)
            {
                motor_check = 0;
            }
            if ((dial_target_angle - dial_instance->message.out_position) > (1.5f*M_PI))
                dial_target_angle -= 2*M_PI;
            if ((dial_instance->message.out_position - dial_target_angle) > (1.5f*M_PI))
                dial_target_angle += 2*M_PI;
            if (dial_instance->output < -300.0f)
            {
                dial_instance->velocity_pid->i_out = 0.0f;
                dial_instance->output = 0.0f;
            }
            Motor_Dji_Control(dial_instance, dial_target_angle);
        }else if ((Et08a->sb == 2)) //连发
        {
            //Motor_Dji_Control(wheel_1_instance, -wheel_velocity);
            //Motor_Dji_Control(wheel_2_instance, -wheel_velocity);
            //Motor_Dji_Control(wheel_3_instance, -wheel_velocity);

            if (Et08a->sa == 1)
            {
                count++;
                if (count == 125-1){
                    //if ((dial_target_angle - dial_instance->message.out_position) < M_PI/2.0f)
                        dial_target_angle += 44.98f/180.0f*M_PI;
                    count=0;
                }
            }
            if ((dial_target_angle - dial_instance->message.out_position) > (1.5f*M_PI))
                dial_target_angle -= 2*M_PI;
            if ((dial_instance->message.out_position - dial_target_angle) > (1.5f*M_PI))
                dial_target_angle += 2*M_PI;
            if (dial_instance->output < -300.0f)
            {
                dial_instance->velocity_pid->i_out = 0.0f;
                dial_instance->output = 0.0f;
            }
            Motor_Dji_Control(dial_instance, dial_target_angle);
        }

        if (Et08a->sb == 2 || Et08a->sb == 3)
        {
            //pitch轴控制
            pitch_target_angle -= Et08a->ly/67000.0f*0.31415f;
            //限位
            if (pitch_target_angle > -0.1f)
                pitch_target_angle = -0.1f;
            else if (pitch_target_angle < -0.9f)
                pitch_target_angle = -0.9f;
            Motor_Dm_Control(pitch_instance, pitch_target_angle);
            //摩擦力补偿
            if (pitch_instance->output < 0.0f)
                pitch_instance->output -= fraction_offset;
            else if (pitch_instance->output > 0.0f)
                pitch_instance->output += fraction_offset;
            Motor_Dm_Mit_Control(pitch_instance, 0, 0, pitch_instance->output - galaxy_offset);
        }

        //状态判断
        if (Et08a->sb == 2 || Et08a->sb == 3)
        {
            Pid_Enable(wheel_1_instance->velocity_pid);
            Pid_Enable(wheel_2_instance->velocity_pid);
            Pid_Enable(wheel_3_instance->velocity_pid);
            Pid_Enable(dial_instance->velocity_pid);
            if (pitch_instance->motor_state == DM_DISABLE)
            {
                Motor_Dm_Cmd(pitch_instance, DM_CMD_MOTOR_ENABLE);
            }
        }else if (Et08a->sb == 1 || Et08a == NULL)
        {
            Motor_Dji_Control(wheel_1_instance, 0);
            Motor_Dji_Control(wheel_2_instance, 0);
            Motor_Dji_Control(wheel_3_instance, 0);
            Motor_Dji_SetCurrent(dial_instance, 0);
            Motor_Dm_Cmd(pitch_instance, DM_CMD_MOTOR_DISABLE);

            dial_target_angle = dial_instance->message.out_position;
        }

        Motor_Dji_Transmit(wheel_1_instance);
        Motor_Dji_Transmit(wheel_2_instance);
        Motor_Dji_Transmit(wheel_3_instance);
        Motor_Dji_Transmit(dial_instance);
        Motor_Dm_Transmit(pitch_instance);

        Referee_Clear_Uart_Error(ref_instance);

        osDelay(1);
    }
}