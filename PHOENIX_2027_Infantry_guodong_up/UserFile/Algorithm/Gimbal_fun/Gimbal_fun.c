//
// Created by 朱泽超 on 2026/11/8.
//

#include "Gimbal_fun.h"

#include <math.h>
#include <stddef.h>

/* ========================= 内部宏 ========================= */
#define GIMBAL_PI 3.14159265358979f

#define GIMBAL_SEQ_IS_FOLD(seq)   ((seq) >= GIMBAL_SEQ_FOLD_SMALL_PITCH && (seq) <= GIMBAL_SEQ_FOLD_DONE)
#define GIMBAL_SEQ_IS_UNFOLD(seq) ((seq) >= GIMBAL_SEQ_UNFOLD_YAW && (seq) <= GIMBAL_SEQ_UNFOLD_DONE)

/* ========================= 内部函数 ========================= */

/**
 * @brief 把角度归一化到(-PI, PI]
 */
float gimbal_angle_wrap(float angle)
{
    while (angle > GIMBAL_PI)
    {
        angle -= 2.0f * GIMBAL_PI;
    }
    while (angle < -GIMBAL_PI)
    {
        angle += 2.0f * GIMBAL_PI;
    }
    return angle;
}

/**
 * @brief 限幅
 */
static float gimbal_clamp(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
}

/**
 * @brief 设置某个轴的目标值, 并限制单周期最大变化量
 * @note 折叠/展开时目标值是阶跃变化的, 直接丢给MIT会给出一个巨大的刚度力矩,
 *       所以这里做斜率限制, 让目标值自己"走"过去
 */
static void gimbal_target_set(gimbal_motor_message_s *motor, float target, float max_step)
{
    const float last = motor->target_position;

    if (max_step > 0.0f)
    {
        if (target > last + max_step)
        {
            target = last + max_step;
        }
        else if (target < last - max_step)
        {
            target = last - max_step;
        }
    }

    motor->target_position = target;
}

/**
 * @brief 推进当前时序步的计时器
 * @param limit_ms 本步的判定时间(到位超时时间 或 纯等待时间)
 * @return true表示已经到达 limit_ms
 */
static bool gimbal_step_timer_run(gimbal_message_s *gimbal_message, uint16_t limit_ms)
{
    if (gimbal_message->seq_timer < limit_ms)
    {
        gimbal_message->seq_timer += (uint16_t)GIMBAL_CONTROL_PERIOD_MS;
    }
    return (gimbal_message->seq_timer >= limit_ms);
}

/**
 * @brief 进入下一个时序步, 计时器清零
 */
static void gimbal_step_next(gimbal_message_s *gimbal_message, gimbal_seq_e next_state)
{
    gimbal_message->seq_state = next_state;
    gimbal_message->seq_timer = 0U;
}

/**
 * @brief 某个轴是否已经到达目标位置
 */
static bool gimbal_axis_in_position(const gimbal_motor_message_s *motor, float target)
{
    return (fabsf(target - motor->current_position) <= GIMBAL_POSITION_TOLERANCE);
}

/**
 * @brief 摇杆累积型目标值: 当前目标 + 摇杆增量, 先限幅再斜率限制
 * @note 顺序很重要: 先限幅再斜率限制, 折叠时"收中位"这种目标值突变才不会
 *       绕过斜率限制直接拍给MIT(那一下会给出一个巨大的刚度力矩)
 */
static void gimbal_target_add(gimbal_motor_message_s *motor, float ch,
                              float min, float max, float max_step)
{
    gimbal_target_set(motor, gimbal_clamp(motor->target_position + ch * SENSITIVITY, min, max),
                      max_step);
}

/**
 * @brief 云台状态函数: 只看s1一个拨杆
 * @param dr16_instance 遥控器实例
 * @return 云台状态
 * @note 遥控器掉线时s1为0, 会走default分支返回GIMBAL_DISABLE, 顺带做了掉线保护
 */
static gimbal_state_e gimbal_state_fun(Dr16Instance_s *dr16_instance)
{
    if (dr16_instance == NULL)
    {
        return GIMBAL_DISABLE;
    }

    switch (dr16_instance->dr16_handle.s1)
    {
        case 1:                                 //拨杆上
            return GIMBAL_NORMAL;
        case 3:                                 //拨杆中
            return GIMBAL_FOLD;
        case 2:                                 //拨杆下
        default:                                //遥控器掉线/数据异常
            return GIMBAL_DISABLE;
    }
}

/**
 *
 * @param dr16_instance
 */
bool gimbal_init_fun(Dr16Instance_s *dr16_instance)
{
    static uint16_t stable_cnt = 0U;

    if (dr16_instance == NULL)
    {
        stable_cnt = 0U;
        return false;
    }

    /* 两个三位开关都要在最上方(值1) */
    if (dr16_instance->dr16_handle.s1 == 1 && dr16_instance->dr16_handle.s2 == 1)
    {
        if (stable_cnt < GIMBAL_INIT_STABLE_MS)
        {
            stable_cnt++;
        }
    }
    else
    {
        stable_cnt = 0U;                        //中途拨下来就从零开始计
    }

    return (stable_cnt >= GIMBAL_INIT_STABLE_MS);
}

/**
 * @brief 折叠时序: 小pitch收回 -> 大pitch折叠 -> yaw锁中位
 */
static void gimbal_fold_fun(gimbal_message_s *gimbal_message, float ch_yaw, float ch_pitch)
{
    /* 从其它流程(展开完成/失能)进来, 从第一步开始 */
    if (GIMBAL_SEQ_IS_FOLD(gimbal_message->seq_state) == false)
    {
        gimbal_step_next(gimbal_message, GIMBAL_SEQ_FOLD_SMALL_PITCH);
    }

    /* yaw一直由摇杆累积; 走到第3步之后收紧到灯条死区里(默认死区是0, 也就是锁中位) */
    if (gimbal_message->seq_state >= GIMBAL_SEQ_FOLD_YAW)
    {
        gimbal_target_add(&gimbal_message->yaw, ch_yaw,
                          YAW_DEADZONE_DOWN, YAW_DEADZONE_UP, YAW_TARGET_STEP);
    }
    else
    {
        gimbal_target_add(&gimbal_message->yaw, ch_yaw,
                          YAW_ANGLE_MIN, YAW_ANGLE_MAX, YAW_TARGET_STEP);
    }

    switch (gimbal_message->seq_state)
    {
        case GIMBAL_SEQ_FOLD_SMALL_PITCH:       //1. 小pitch先收回到折叠位, 另外两个轴保持不动
            gimbal_target_set(&gimbal_message->small_pitch,
                              SMALL_PITCH_FOLD_POSITION, SMALL_PITCH_TARGET_STEP);
            if (gimbal_axis_in_position(&gimbal_message->small_pitch, SMALL_PITCH_FOLD_POSITION))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_FOLD_BIG_PITCH);
            }
            else if (gimbal_step_timer_run(gimbal_message, GIMBAL_STEP_TIMEOUT_MS))
            {
                gimbal_message->seq_timeout = true;     //小pitch收不回来, 不能再折大pitch
            }
            break;

        case GIMBAL_SEQ_FOLD_BIG_PITCH:         //2. 小pitch保持收回, 大pitch折叠
            gimbal_target_set(&gimbal_message->small_pitch,
                              SMALL_PITCH_FOLD_POSITION, SMALL_PITCH_TARGET_STEP);
            gimbal_target_set(&gimbal_message->big_pitch,
                              BIG_PITCH_FOLD_POSITION, BIG_PITCH_TARGET_STEP);
            if (gimbal_axis_in_position(&gimbal_message->big_pitch, BIG_PITCH_FOLD_POSITION))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_FOLD_YAW);
            }
            else if (gimbal_step_timer_run(gimbal_message, GIMBAL_STEP_TIMEOUT_MS))
            {
                gimbal_message->seq_timeout = true;
            }
            break;

        case GIMBAL_SEQ_FOLD_YAW:               //3. yaw被死区限幅收到中位, 两个pitch保持
            if (gimbal_step_timer_run(gimbal_message, GIMBAL_SEQ_DWELL_MS))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_FOLD_DONE);
            }
            break;

        case GIMBAL_SEQ_FOLD_DONE:              //4. 折叠完成, 保持折叠姿态
        default:
            gimbal_target_set(&gimbal_message->big_pitch,
                              BIG_PITCH_FOLD_POSITION, BIG_PITCH_TARGET_STEP);
#if (SMALL_PITCH_FOLLOW_IN_FOLD != 0)
            /* 折叠后小pitch仍可跟随ch1(原逻辑); 不希望它动就把 Gimbal_fun.h 里的宏置0 */
            gimbal_target_add(&gimbal_message->small_pitch, ch_pitch,
                              SMALL_PITCH_ANGLE_MIN, SMALL_PITCH_ANGLE_MAX,
                              SMALL_PITCH_TARGET_STEP);
#else
            gimbal_target_set(&gimbal_message->small_pitch,
                              SMALL_PITCH_FOLD_POSITION, SMALL_PITCH_TARGET_STEP);
            (void)ch_pitch;
#endif
            break;
    }
}

/**
 * @brief 展开时序(折叠的反向): yaw解锁 -> 大pitch展开 -> 小pitch伸出
 */
static void gimbal_unfold_fun(gimbal_message_s *gimbal_message, float ch_yaw, float ch_pitch)
{
    /* 从其它流程(折叠完成/失能)进来, 从第一步开始 */
    if (GIMBAL_SEQ_IS_UNFOLD(gimbal_message->seq_state) == false)
    {
        gimbal_step_next(gimbal_message, GIMBAL_SEQ_UNFOLD_YAW);
    }

    /* 展开过程中yaw按正常模式的机械限位走 */
    gimbal_target_add(&gimbal_message->yaw, ch_yaw,
                      YAW_ANGLE_MIN, YAW_ANGLE_MAX, YAW_TARGET_STEP);

    switch (gimbal_message->seq_state)
    {
        case GIMBAL_SEQ_UNFOLD_YAW:             //1. yaw先解锁交给操作手, 两个pitch就地保持
            if (gimbal_step_timer_run(gimbal_message, GIMBAL_SEQ_DWELL_MS))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_UNFOLD_BIG_PITCH);
            }
            break;

        case GIMBAL_SEQ_UNFOLD_BIG_PITCH:       //2. 大pitch展开到正常位, 小pitch还是收着
            gimbal_target_set(&gimbal_message->big_pitch,
                              BIG_PITCH_NORMAL_POSITION, BIG_PITCH_TARGET_STEP);
            if (gimbal_axis_in_position(&gimbal_message->big_pitch, BIG_PITCH_NORMAL_POSITION))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_UNFOLD_SMALL_PITCH);
            }
            else if (gimbal_step_timer_run(gimbal_message, GIMBAL_STEP_TIMEOUT_MS))
            {
                gimbal_message->seq_timeout = true;
            }
            break;

        case GIMBAL_SEQ_UNFOLD_SMALL_PITCH:     //3. 大pitch保持, 小pitch伸出到正常位
            gimbal_target_set(&gimbal_message->big_pitch,
                              BIG_PITCH_NORMAL_POSITION, BIG_PITCH_TARGET_STEP);
            gimbal_target_set(&gimbal_message->small_pitch,
                              SMALL_PITCH_NORMAL_POSITION, SMALL_PITCH_TARGET_STEP);
            if (gimbal_axis_in_position(&gimbal_message->small_pitch, SMALL_PITCH_NORMAL_POSITION))
            {
                gimbal_step_next(gimbal_message, GIMBAL_SEQ_UNFOLD_DONE);
            }
            else if (gimbal_step_timer_run(gimbal_message, GIMBAL_STEP_TIMEOUT_MS))
            {
                gimbal_message->seq_timeout = true;
            }
            break;

        case GIMBAL_SEQ_UNFOLD_DONE:            //4. 展开完成, 进入正常操作
        default:
            gimbal_target_set(&gimbal_message->big_pitch,
                              BIG_PITCH_NORMAL_POSITION, BIG_PITCH_TARGET_STEP);
            gimbal_target_add(&gimbal_message->small_pitch, ch_pitch,
                              SMALL_PITCH_ANGLE_MIN, SMALL_PITCH_ANGLE_MAX,
                              SMALL_PITCH_TARGET_STEP);
            break;
    }
}

/**
 * @brief 云台控制计算函数
 * @return
 */
void gimbal_control_fun(gimbal_message_s *gimbal_message)
{
    if (gimbal_message == NULL)
    {
        return;
    }

    Dr16Instance_s *dr16_instance = gimbal_message->dr16_instance;

    const float ch_yaw = (dr16_instance != NULL) ? (float)dr16_instance->dr16_handle.ch0 : 0.0f;
    const float ch_pitch = (dr16_instance != NULL) ? (float)dr16_instance->dr16_handle.ch1 : 0.0f;

    /* ---------- 1. 上电解锁: 两个拨杆拨到双上并保持一段时间 ---------- */
    if (gimbal_message->gimbal_init_state == 0U)
    {
        if (gimbal_init_fun(dr16_instance))
        {
            gimbal_message->gimbal_init_state = 1U;
        }
    }

    /* ---------- 2. 状态解析 ---------- */
    const gimbal_state_e raw_state = gimbal_state_fun(dr16_instance);

    /* 拨杆回下: 清掉时序超时, 允许重新来一次 */
    if (raw_state == GIMBAL_DISABLE)
    {
        gimbal_message->seq_timeout = false;
    }

    if (gimbal_message->gimbal_init_state == 0U || gimbal_message->seq_timeout != false)
    {
        gimbal_message->gimbal_state = GIMBAL_DISABLE;
    }
    else
    {
        gimbal_message->gimbal_state = raw_state;
    }

    /* ---------- 3. 按状态给三个轴的目标值 ---------- */
    if (gimbal_message->gimbal_state == GIMBAL_DISABLE)  //电机全设失能
    {
        gimbal_message->yaw.gimbal_motor_state = false;
        gimbal_message->big_pitch.gimbal_motor_state = false;
        gimbal_message->small_pitch.gimbal_motor_state = false;

        /* 目标值直接跟住当前反馈, 这样重新使能时不会有阶跃 */
        gimbal_message->yaw.target_position = gimbal_message->yaw.current_position;
        gimbal_message->big_pitch.target_position = gimbal_message->big_pitch.current_position;
        gimbal_message->small_pitch.target_position = gimbal_message->small_pitch.current_position;

        /* 时序归零, 下次使能从第一步重新走 */
        gimbal_message->seq_state = GIMBAL_SEQ_NONE;
        gimbal_message->seq_timer = 0U;
    }

    if (gimbal_message->gimbal_state == GIMBAL_NORMAL)   //电机全设使能
    {
        gimbal_message->yaw.gimbal_motor_state = true;
        gimbal_message->big_pitch.gimbal_motor_state = true;
        gimbal_message->small_pitch.gimbal_motor_state = true;

        gimbal_unfold_fun(gimbal_message, ch_yaw, ch_pitch);
    }

    if (gimbal_message->gimbal_state == GIMBAL_FOLD)     //折叠时yaw轴设置死区
    {
        gimbal_message->yaw.gimbal_motor_state = true;
        gimbal_message->big_pitch.gimbal_motor_state = true;
        gimbal_message->small_pitch.gimbal_motor_state = true;

        gimbal_fold_fun(gimbal_message, ch_yaw, ch_pitch);
    }
}

/**
 * @brief 取状态名字符串
 */
const char *gimbal_state_name(gimbal_state_e state)
{
    switch (state)
    {
        case GIMBAL_NORMAL:
            return "NORMAL";
        case GIMBAL_FOLD:
            return "FOLD";
        case GIMBAL_DISABLE:
        default:
            return "DISABLE";
    }
}
