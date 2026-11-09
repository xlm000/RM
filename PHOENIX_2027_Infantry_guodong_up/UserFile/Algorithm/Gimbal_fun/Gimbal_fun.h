//
// Created by 朱泽超 on 2026/11/8.
//

#ifndef PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H
#define PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H

#include "dev_dr16.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================= 遥控器手感 ========================= */
#define SENSITIVITY (0.001f / 660.0f)   //遥控器灵敏度定义(按1ms控制周期算, 满杆约1rad/s)

/* ========================= 机械角度(rad) ========================= */
#define BIG_PITCH_NORMAL_POSITION   0.0f    //大pitch垂直位置
#define BIG_PITCH_FOLD_POSITION     1.0f    //大pitch折叠位置
#define SMALL_PITCH_NORMAL_POSITION 0.0f    //小pitch正常(伸出)位置
#define SMALL_PITCH_FOLD_POSITION   0.0f    //小pitch折叠(收回)位置

/* ========================= 角度限幅(rad) ========================= */
#define YAW_ANGLE_MAX 2.80f                 //正常模式下yaw目标上限, 必须 < PI
#define YAW_ANGLE_MIN (-2.80f)              //正常模式下yaw目标下限
#define SMALL_PITCH_ANGLE_MAX 1.40f         //小pitch目标上限
#define SMALL_PITCH_ANGLE_MIN (-1.40f)      //小pitch目标下限

/* 折叠时yaw的"灯条死区": 上限必须大于下限才生效;
 * 两者相等(默认都是0)表示死区是一个点, 折叠时yaw直接锁在中位 */
#define YAW_DEADZONE_UP   0.0f              //yaw灯条死区上限
#define YAW_DEADZONE_DOWN 0.0f              //yaw灯条死区下限, UP>DOWN

/* ========================= 时序/到位判定 ========================= */
#define GIMBAL_CONTROL_PERIOD_MS   1U       //控制周期(ms), 必须和任务里的osDelay一致
#define GIMBAL_INIT_STABLE_MS      200U     //双上保持多久才算上电解锁(ms)
#define GIMBAL_STEP_TIMEOUT_MS     2000U    //折叠/展开单步超时(ms), 超时后云台失能
#define GIMBAL_SEQ_DWELL_MS        200U     //yaw锁中位/解锁这类"纯等待"步骤的保持时间(ms)
/* 到位判定阈值(rad): MIT是纯刚度控制, 没有积分, 重力负载下会有稳态误差
 * 误差≈负载力矩/kp, 所以 kp 给太小会一直判不到位 -> 超时 -> 失能。
 * 判不到位的时候先把 kp 往上加, 或者把这个阈值放大 */
#define GIMBAL_POSITION_TOLERANCE  0.05f

/* ========================= 目标值斜率限制(rad/控制周期) ========================= */
#define BIG_PITCH_TARGET_STEP    0.004f     //大pitch目标值单周期最大变化量
#define SMALL_PITCH_TARGET_STEP  0.004f     //小pitch目标值单周期最大变化量
#define YAW_TARGET_STEP          0.010f     //yaw目标值单周期最大变化量

/* 折叠完成后小pitch是否还跟随ch1: 1=跟随, 0=锁在折叠位 */
#define SMALL_PITCH_FOLLOW_IN_FOLD 1

//云台状态枚举
typedef enum
{
    GIMBAL_DISABLE = 0U,  //失能
    GIMBAL_NORMAL = 1U,   //正常模式
    GIMBAL_FOLD = 2U      //折叠模式
}gimbal_state_e;

//折叠/展开时序状态
typedef enum
{
    GIMBAL_SEQ_NONE = 0,            //未开始(失能时)

    GIMBAL_SEQ_FOLD_SMALL_PITCH,    //折叠1: 小pitch收回
    GIMBAL_SEQ_FOLD_BIG_PITCH,      //折叠2: 大pitch折叠
    GIMBAL_SEQ_FOLD_YAW,            //折叠3: yaw收到死区内(锁中位)
    GIMBAL_SEQ_FOLD_DONE,           //折叠完成

    GIMBAL_SEQ_UNFOLD_YAW,          //展开1: yaw解锁(先让操作手接管yaw)
    GIMBAL_SEQ_UNFOLD_BIG_PITCH,    //展开2: 大pitch展开到正常位
    GIMBAL_SEQ_UNFOLD_SMALL_PITCH,  //展开3: 小pitch伸出到正常位
    GIMBAL_SEQ_UNFOLD_DONE,         //展开完成, 进入正常操作
}gimbal_seq_e;

//云台电机控制信息
typedef struct
{
    bool  gimbal_motor_state;  //0失能，1使能
    float target_position;     //目标位置(rad)
    float current_position;    //当前反馈位置(rad), 由GimbalTask每周期填入
}gimbal_motor_message_s;

//云台控制信息汇总
typedef struct
{
    Dr16Instance_s *dr16_instance;       //遥控器实例

    uint8_t gimbal_init_state;           //云台初始化状态，上电后两个拨杆必须先拨至双上才可继续操作，0为未初始化，1为已初始化
    gimbal_state_e gimbal_state;         //云台状态

    gimbal_seq_e seq_state;              //折叠/展开时序状态
    uint16_t seq_timer;                  //当前时序步已经过的时间(ms)
    bool seq_timeout;                    //时序超时标志, 置位后云台失能(拨杆拨下可复位)
    uint8_t yaw_feedback_source;         //yaw反馈来源: 0=电机编码器, 1=BMI088陀螺仪

    gimbal_motor_message_s yaw;          //yaw轴控制信息
    gimbal_motor_message_s big_pitch;    //大pitch轴控制信息
    gimbal_motor_message_s small_pitch;  //小pitch轴控制信息
}gimbal_message_s;

/**
 * @brief 上电解锁判断: 两个拨杆都拨到最上方并保持 GIMBAL_INIT_STABLE_MS 才返回true
 * @param dr16_instance 遥控器实例
 * @return true表示已解锁
 */
bool gimbal_init_fun(Dr16Instance_s *dr16_instance);

/**
 * @brief 云台控制计算函数(1kHz调用): 解析状态 -> 推进折叠/展开时序 -> 算三个轴的目标位置
 * @param gimbal_message 云台信息汇总, 调用前需要填好三个轴的 current_position
 * @note 函数只算目标值, 不碰电机; 电机使能/下发在GimbalTask里做
 */
void gimbal_control_fun(gimbal_message_s *gimbal_message);

/**
 * @brief 把角度归一化到(-PI, PI]
 */
float gimbal_angle_wrap(float angle);

/**
 * @brief 取状态名字符串, 调试打印用
 */
const char *gimbal_state_name(gimbal_state_e state);

#endif //PHOENIX_2027_INFANTRY_GUODONG_UP_GIMBAL_FUN_H
