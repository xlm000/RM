//
// Created by 朱泽超 on 2026/11/8.
//

#ifndef PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBALTASK_H
#define PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBALTASK_H

#include "Gimbal_fun.h"

/**
 * @brief 云台任务(CubeMX里创建的Gimbal线程), 1kHz
 * @note 覆盖了 freertos.c 里的 __weak GimbalTask
 */
void GimbalTask(void const * argument);

/* 云台控制信息汇总, 其它任务(比如发射)可以读它拿云台状态/角度 */
extern gimbal_message_s gimbal_message;

/* 遥控器实例, 其它任务可以直接用 */
extern Dr16Instance_s *dr16_instance;

#endif //PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBALTASK_H
