//
// Created by 朱泽超 on 2026/11/8.
//

#include "GimbalTask.h"
#include "Gimbal_fun.h"

#include "dev_dr16.h"
#include "dev_motor_dm.h"
#include "dev_motor_rs.h"

#include "bsp_can.h"
#include "bsp_log.h"

#include "cmsis_os.h"
#include "usb_device.h"

#include <math.h>
#include <stddef.h>

/* ============================================================================
 *                                配置区
 * ==========================================================================*/

/* 是否用 BMI088 陀螺仪做 yaw 轴的绝对角度反馈
 *   1: 用陀螺仪(需要 INS 任务在跑, 且 Quater.ins_ready 为 1)
 *   0: 只用电机自身编码器
 * 注意: 即使这里是 1, 只要 INS 还没就绪(没建任务/没调通), 也会自动回退到编码器,
 *       并且换反馈源的那一帧会把 yaw 目标重新对齐, 不会因为换系而甩一下 */
#define GIMBAL_USE_IMU_YAW          1

/* 是否在云台任务里顺手把 INS 任务(isttask)建出来
 * ！！！打开前必须已经在 CubeMX 里配好: SPI1(PA5/PA6/PA7)、IMU 片选(PA4/PB0)、
 *     加热 PWM(TIM10), 否则 Bmi088_Register 失败会进 Error_Handler 卡死。
 *     已经在 CubeMX 的 FREERTOS 里加过 isttask 的, 这里必须保持 0 */
#define GIMBAL_ENABLE_INS_TASK      0

/* 周期性打印云台状态(J-Link RTT), 调时序的时候打开 */
#define GIMBAL_DEBUG_LOG            0

#if (GIMBAL_USE_IMU_YAW != 0) || (GIMBAL_ENABLE_INS_TASK != 0)
#include "app_ins_task.h"
#endif

/* ============================================================================
 *                            电机参数配置
 * ==========================================================================*/

/* ---- MIT 刚度/阻尼 ----
 * 调参顺序: 先把云台拿在手上, kp 从 0 一点点往上加到能托住自重且不往下掉,
 *           再加 kd 抑制抖动(太大电机会发烫/嗡鸣)。
 * DM 电机 kp 上限 500 / kd 上限 5, RS 电机 kp 上限 500 / kd 上限 5 */
#define YAW_MIT_KP          30.0f
#define YAW_MIT_KD          1.0f
#define BIG_PITCH_MIT_KP    30.0f
#define BIG_PITCH_MIT_KD    1.0f
#define SMALL_PITCH_MIT_KP  20.0f
#define SMALL_PITCH_MIT_KD  0.5f

/* ---- 反馈/给定方向修正 ----
 * 内部所有逻辑(限位/到位判断)都在"云台坐标系"里算:
 *      读反馈:  current_position = DIR * 电机反馈
 *      下发时:  电机目标     = DIR * target_position
 * DIR = +1 电机正方向就是云台正方向; -1 反过来(机械装反了才改)。
 * 注意: dev_motor_rs 的 Motor_Rs_Decode 把位置取了反, 而 Motor_RS_Mit_Control
 *       下发时没有取反, 所以小pitch读反馈的地方先手动补一个负号把驱动这一层
 *       的符号掰回来(见下面 RS_FEEDBACK_SIGN), 这样三轴就能统一用 DIR 了。 */
#define RS_FEEDBACK_SIGN    (-1.0f)     //补偿 dev_motor_rs 解码里的取反, 别乱改
#define YAW_DIR             1.0f
#define BIG_PITCH_DIR       1.0f
#define SMALL_PITCH_DIR     1.0f

/* ---- CAN 总线号(三个电机都在总线上, 1=CAN1) ---- */
#define GIMBAL_CAN_NUMBER   1

/* ============================================================================
 *                          电机注册配置
 * ==========================================================================*/
/* ！！！下面四个 ID 全都是占位的 TODO, 一定要按你实际车上电机的拨码/上位机设置改！！！
 *   DM 电机: tx_id = 电机 CAN ID(下发给电机的ID), rx_id = 反馈ID(一般是 0x10 + 电机ID)
 *   RS 电机: tx_id = 电机 ID,                          rx_id = 主机ID(默认 0xFD)   */

/* yaw轴: 达妙 J4310 */
static DmMotorInitConfig_s yaw_motor_config = {
        .topic_name = "gimbal_yaw",
        .type = J4310,
        .parameters = {
                /* ！！！P_MAX/V_MAX/T_MAX 必须和达妙上位机里设置的完全一致,
                 * MIT 帧的位置/速度/力矩都是按这三个值做 16/12 位编解码的 */
                .pos_max = 12.5f,
                .vel_max = 30.0f,
                .tor_max = 10.0f,
                .kp_max  = 500.0f,
                .kd_max  = 5.0f,
                .kp_int  = 0.0f,        /* MIT 路径下不用这两个, kp/kd 下发时显式给 */
                .kd_int  = 0.0f,
        },
        .control_mode = DM_POSITION,     /* 走MIT, 这个字段只是占位 */
        .can_config = {
                .can_number = GIMBAL_CAN_NUMBER,
                .tx_id = 0x01,           /* TODO 电机ID */
                .rx_id = 0x11,           /* TODO 反馈ID */
        },
        .angle_pid_config    = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 0.0f },
        .velocity_pid_config = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 0.0f },
};

/* 大pitch轴: 达妙 J4340 */
static DmMotorInitConfig_s big_pitch_motor_config = {
        .topic_name = "gimbal_big_pitch",
        .type = J4340,
        .parameters = {
                .pos_max = 12.5f,
                .vel_max = 10.0f,        /* J4340 的 V_MAX 一般是 10 */
                .tor_max = 28.0f,        /* J4340 的 T_MAX 一般是 28 */
                .kp_max  = 500.0f,
                .kd_max  = 5.0f,
                .kp_int  = 0.0f,
                .kd_int  = 0.0f,
        },
        .control_mode = DM_POSITION,
        .can_config = {
                .can_number = GIMBAL_CAN_NUMBER,
                .tx_id = 0x02,           /* TODO 电机ID */
                .rx_id = 0x12,           /* TODO 反馈ID */
        },
        .angle_pid_config    = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 0.0f },
        .velocity_pid_config = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 0.0f },
};

/* 小pitch轴: 灵足 EL05 */
static RsMotorInitConfig_s small_pitch_motor_config = {
        .topic_name = "gimbal_small_pitch",
        .type = EL_05,
        .can_config = {
                .can_number = GIMBAL_CAN_NUMBER,
                .tx_id = 0x03,           /* TODO 电机ID */
                .rx_id = 0xFD,           /* TODO 主机ID(RS默认0xFD) */
        },
        .control_mode = RS_POSITION,     /* 走MIT, 这个字段只是占位 */
        .angle_pid_config    = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 6.0f },
        .velocity_pid_config = { .kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .out_max = 50.0f },
};

/* ============================================================================
 *                              全局实例
 * ==========================================================================*/
Dr16Instance_s *dr16_instance;

static DmMotorInstance_s *yaw_motor;          //yaw轴电机
static DmMotorInstance_s *big_pitch_motor;    //大pitch轴电机
static RsMotorInstance_s *small_pitch_motor;  //小pitch轴电机

gimbal_message_s gimbal_message;              //云台控制信息汇总

/* 电机使能/失能只在状态变化时发一帧命令, 避免命令帧把总线刷满 */
static bool yaw_motor_applied;
static bool big_pitch_motor_applied;
static bool small_pitch_motor_applied;

/* ============================================================================
 *                            内部函数
 * ==========================================================================*/

/**
 * @brief 实例注册 + 初始反馈对齐
 * @return true注册成功
 * @note 这里不使能电机, 使能交给状态机(上电默认失能, 拨杆双上之后才会使能)
 */
static bool gimbal_register_fun(void)
{
    /* 遥控器实例 */
    dr16_instance = Dr16_Register(&huart6);
    if (dr16_instance == NULL)
    {
        Log_Error("gimbal: dr16 register failed");
        return false;
    }
    gimbal_message.dr16_instance = dr16_instance;   //遥控器先挂上, 后面电机注册失败也能继续解析状态

    /* 电机实例 */
    yaw_motor = Motor_DM_Register(&yaw_motor_config);
    big_pitch_motor = Motor_DM_Register(&big_pitch_motor_config);
    small_pitch_motor = Motor_RS_Register(&small_pitch_motor_config);

    if (yaw_motor == NULL || big_pitch_motor == NULL || small_pitch_motor == NULL)
    {
        Log_Error("gimbal: motor register failed");
        return false;
    }

    /* 云台信息汇总初始化 */
    gimbal_message.gimbal_init_state = 0U;
    gimbal_message.gimbal_state = GIMBAL_DISABLE;
    gimbal_message.seq_state = GIMBAL_SEQ_NONE;
    gimbal_message.seq_timer = 0U;
    gimbal_message.seq_timeout = false;
    gimbal_message.yaw_feedback_source = 0U;

    /* 三个轴的目标值/反馈先对齐成当前值, 避免第一次使能时目标值是0而位置不是0 */
    gimbal_message.yaw.gimbal_motor_state = false;
    gimbal_message.big_pitch.gimbal_motor_state = false;
    gimbal_message.small_pitch.gimbal_motor_state = false;

    gimbal_message.yaw.current_position = YAW_DIR * yaw_motor->message.out_position;
    gimbal_message.big_pitch.current_position = BIG_PITCH_DIR * big_pitch_motor->message.out_position;
    gimbal_message.small_pitch.current_position =
            SMALL_PITCH_DIR * RS_FEEDBACK_SIGN * small_pitch_motor->message.position;

    gimbal_message.yaw.target_position = gimbal_message.yaw.current_position;
    gimbal_message.big_pitch.target_position = gimbal_message.big_pitch.current_position;
    gimbal_message.small_pitch.target_position = gimbal_message.small_pitch.current_position;

    yaw_motor_applied = false;
    big_pitch_motor_applied = false;
    small_pitch_motor_applied = false;

    Log_Passing("gimbal: register success");
    return true;
}

/**
 * @brief 读取三个轴的反馈位置(rad), 每周期调用一次
 */
static void gimbal_feedback_fun(void)
{
    uint8_t yaw_source = 0U;
    float yaw_feedback = 0.0f;

#if (GIMBAL_USE_IMU_YAW != 0)
    /* 陀螺仪解算出来的yaw是弧度制, 范围(-PI, PI], 精度和零漂取决于INS任务 */
    if (Quater.ins_ready != 0U)
    {
        yaw_source = 1U;
        yaw_feedback = Quater.yaw;
    }
#endif

    /* 陀螺仪不可用(没建INS任务/还没收敛)就退回电机编码器 */
    if (yaw_source == 0U && yaw_motor != NULL)
    {
        yaw_feedback = yaw_motor->message.out_position;
    }

    gimbal_message.yaw.current_position = YAW_DIR * gimbal_angle_wrap(yaw_feedback);

    /* 换反馈源(编码器 <-> 陀螺仪)时, 两套坐标系的零点不一样, 目标值必须重新对齐 */
    if (gimbal_message.yaw_feedback_source != yaw_source)
    {
        gimbal_message.yaw_feedback_source = yaw_source;
        gimbal_message.yaw.target_position = gimbal_message.yaw.current_position;

        if (yaw_source != 0U)
        {
            Log_Passing("gimbal: yaw feedback -> BMI088");
        }
        else
        {
            Log_Warning("gimbal: yaw feedback -> encoder");
        }
    }

    if (big_pitch_motor != NULL)
    {
        gimbal_message.big_pitch.current_position =
                BIG_PITCH_DIR * big_pitch_motor->message.out_position;
    }

    if (small_pitch_motor != NULL)
    {
        gimbal_message.small_pitch.current_position =
                SMALL_PITCH_DIR * RS_FEEDBACK_SIGN * small_pitch_motor->message.position;
    }
}

/**
 * @brief 大pitch/yaw(DM电机)输出: 使能状态切换 + MIT控制帧
 */
static void gimbal_dm_output_fun(DmMotorInstance_s *motor,
                                 const gimbal_motor_message_s *motor_message,
                                 float dir, float kp, float kd, bool *applied)
{
    if (motor == NULL)
    {
        return;
    }

    /* 使能/失能切换: 只发一帧命令, 然后本周期不再发控制帧(让命令帧先上总线) */
    if (motor_message->gimbal_motor_state != *applied)
    {
        *applied = motor_message->gimbal_motor_state;

        Motor_Dm_Cmd(motor, (*applied) ? DM_CMD_MOTOR_ENABLE : DM_CMD_MOTOR_DISABLE);
        Motor_Dm_Transmit(motor);
        return;
    }

    if (*applied == false)
    {
        return;                 //失能状态不发控制帧
    }

    /* 纯位置MIT: 速度/力矩前馈先都留0, 需要的话再往里加 */
    if (Motor_Dm_FullParam_MIT_Control(motor, dir * motor_message->target_position,
                                       0.0f, kp, kd, 0.0f) != false)
    {
        Motor_Dm_Transmit(motor);
    }
}

/**
 * @brief 小pitch(RS电机)输出: 使能状态切换 + MIT控制帧
 */
static void gimbal_rs_output_fun(RsMotorInstance_s *motor,
                                 const gimbal_motor_message_s *motor_message,
                                 float dir, float kp, float kd, bool *applied)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor_message->gimbal_motor_state != *applied)
    {
        *applied = motor_message->gimbal_motor_state;

        if (*applied != false)
        {
            Motor_RS_Enable(motor);
        }
        else
        {
            Motor_RS_Disable(motor);
        }
        Motor_RS_Transmit(motor);
        return;
    }

    if (*applied == false)
    {
        return;
    }

    if (Motor_RS_Mit_Control(motor, dir * motor_message->target_position,
                             0.0f, kp, kd, 0.0f) != false)
    {
        Motor_RS_Transmit(motor);
    }
}

/**
 * @brief 三个轴的目标值下发, 每周期调用一次
 */
static void gimbal_output_fun(void)
{
    gimbal_dm_output_fun(yaw_motor, &gimbal_message.yaw,
                         YAW_DIR, YAW_MIT_KP, YAW_MIT_KD, &yaw_motor_applied);

    gimbal_dm_output_fun(big_pitch_motor, &gimbal_message.big_pitch,
                         BIG_PITCH_DIR, BIG_PITCH_MIT_KP, BIG_PITCH_MIT_KD,
                         &big_pitch_motor_applied);

    gimbal_rs_output_fun(small_pitch_motor, &gimbal_message.small_pitch,
                         SMALL_PITCH_DIR, SMALL_PITCH_MIT_KP, SMALL_PITCH_MIT_KD,
                         &small_pitch_motor_applied);
}

#if (GIMBAL_DEBUG_LOG != 0)
/**
 * @brief 周期性打印状态(约500ms一次), RTT只支持整数, 角度按千分之一弧度打印
 */
static void gimbal_debug_log_fun(void)
{
    static uint16_t log_cnt = 0U;

    if (++log_cnt < 500U)
    {
        return;
    }
    log_cnt = 0U;

    Log_Information("gimbal %s seq:%d yaw:%d big:%d small:%d init:%d tmo:%d",
                    gimbal_state_name(gimbal_message.gimbal_state),
                    (int)gimbal_message.seq_state,
                    (int)(gimbal_message.yaw.current_position * 1000.0f),
                    (int)(gimbal_message.big_pitch.current_position * 1000.0f),
                    (int)(gimbal_message.small_pitch.current_position * 1000.0f),
                    (int)gimbal_message.gimbal_init_state,
                    (int)gimbal_message.seq_timeout);
}
#endif

void GimbalTask(void const * argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN GimbalTask */

#if (GIMBAL_ENABLE_INS_TASK != 0)
    /* 把INS任务建出来(陀螺仪解算), 优先级比云台略高 */
    osThreadDef(Ins, isttask, osPriorityAboveNormal, 0, 1024);
    osThreadCreate(osThread(Ins), NULL);
#endif

    /* 实例注册区: 注册失败就只打日志, 不卡死, 方便重新烧录/排查 */
    if (gimbal_register_fun() == false)
    {
        Log_Error("gimbal: init failed, motors stay disabled");
    }

    /* Infinite loop */
    for(;;)
    {
        /* 1. 读反馈: 三个轴的当前位置 */
        gimbal_feedback_fun();

        /* 2. 算法: 状态解析 + 折叠/展开时序 + 目标位置计算 */
        gimbal_control_fun(&gimbal_message);

        /* 3. 下发: 使能切换 + MIT控制帧 */
        gimbal_output_fun();

#if (GIMBAL_DEBUG_LOG != 0)
        gimbal_debug_log_fun();
#endif

        osDelay(GIMBAL_CONTROL_PERIOD_MS);   //1kHz, 和 Gimbal_fun.h 里的周期定义保持一致
    }
    /* USER CODE END GimbalTask */
}
