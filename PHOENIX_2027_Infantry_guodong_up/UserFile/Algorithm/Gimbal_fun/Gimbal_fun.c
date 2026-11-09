//
// Created by 朱泽超 on 2026/11/8.
//

#include "Gimbal_fun.h"

/**
 * @brief 云台状态函数
 * @return
 */
static gimbal_state_e gimbal_state_fun(Dr16Instance_s *dr16_instance)
{
    if (dr16_instance->dr16_handle.s1 == 1)
    {

    }
    if (dr16_instance->dr16_handle.s1 == 1)
    {

    }
    if (dr16_instance->dr16_handle.s1 == 1)
    {

    }
    else
    {
        return GIMBAL_DISABLE;
    }
}

/**
 *
 * @param dr16_instance
 */
bool gimbal_init_fun(Dr16Instance_s *dr16_instance)
{

}

/**
 * @brief 云台控制计算函数
 * @return
 */
void gimbal_control_fun(gimbal_message_s *gimbal_message)
{
    gimbal_message->gimbal_state = gimbal_state_fun(gimbal_message->dr16_instance);

    if (gimbal_message->gimbal_state == GIMBAL_DISABLE)  //电机全设失能
    {
        gimbal_message->yaw.gimbal_motor_state = 0U;
        gimbal_message->big_pitch.gimbal_motor_state = 0U;
        gimbal_message->small_pitch.gimbal_motor_state = 0U;
    }
    if (gimbal_message->gimbal_state == GIMBAL_NORMAL)   //电机全设使能
    {
        gimbal_message->yaw.gimbal_motor_state = 1U;
        gimbal_message->big_pitch.gimbal_motor_state = 1U;
        gimbal_message->small_pitch.gimbal_motor_state = 1U;

        gimbal_message->big_pitch.target_position = BIG_PITCH_NORMAL_POSITION;
        gimbal_message->yaw.target_position += gimbal_message->dr16_instance->dr16_handle.ch0 * SENSITIVITY;
        gimbal_message->small_pitch.target_position += gimbal_message->dr16_instance->dr16_handle.ch1 * SENSITIVITY;
    }
    if (gimbal_message->gimbal_state == GIMBAL_FOLD)
    {
        gimbal_message->yaw.gimbal_motor_state = 1U;
        gimbal_message->big_pitch.gimbal_motor_state = 1U;
        gimbal_message->small_pitch.gimbal_motor_state = 1U;

        gimbal_message->big_pitch.target_position = BIG_PITCH_FOLD_POSITION;
        gimbal_message->yaw.target_position += gimbal_message->dr16_instance->dr16_handle.ch0 * SENSITIVITY;
        if (gimbal_message->yaw.target_position)
        gimbal_message->small_pitch.target_position += gimbal_message->dr16_instance->dr16_handle.ch1 * SENSITIVITY;
    }
}