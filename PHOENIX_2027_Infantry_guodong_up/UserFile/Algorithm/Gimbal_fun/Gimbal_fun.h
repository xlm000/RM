//
// Created by 朱泽超 on 2026/11/8.
//

#ifndef PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H
#define PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H

#include "dev_dr16.h"
#include <stdbool.h>

#define SENSITIVITY 0.001f/660.0f       //遥控器灵敏度定义
#define BIG_PITCH_NORMAL_POSITION 0.0f  //大pitch垂直位置
#define BIG_PITCH_FOLD_POSITION 1.0f    //大pitch折叠位置
#define YAW_DEADZONE_UP 0.0f            //yaw灯条死区上限
#define YAW_DEADZONE_DOWN 0.0f          //yaw灯条死区下限

//云台状态枚举
typedef enum
{
    GIMBAL_DISABLE = 0U,  //失能
    GIMBAL_NORMAL = 1U,   //正常模式
    GIMBAL_FOLD = 2U      //折叠模式
}gimbal_state_e;

//云台电机控制信息
typedef struct
{
    bool  gimbal_motor_state;  //0失能，1使能
    float target_position;
}gimbal_motor_message_s;

//云台控制信息汇总
typedef struct
{
    Dr16Instance_s *dr16_instance;       //遥控器实例

    uint8_t gimbal_init_state;           //云台初始化状态，上电后两个拨杆必须先拨至双上才可继续操作，0为未初始化，1为已初始化
    gimbal_state_e gimbal_state;         //云台状态


    gimbal_motor_message_s yaw;          //yaw轴控制信息
    gimbal_motor_message_s big_pitch;    //大pitch轴控制信息
    gimbal_motor_message_s small_pitch;  //小pitch轴控制信息
}gimbal_message_s;

#endif //PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H
