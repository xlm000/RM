/** 
 *  @file dev_motor_dji.c
 *  @brief 大疆电机控制模块
 *  @details 大疆电机控制模块，提供大疆电机初始化，控制等功能
 *  @date 2025-07-04
 */
#include "dev_motor_dji.h"
#include "robot_config.h"


static void Motor_Dji_Decode(CanInstance_s *can_instance);
static uint8_t idx; 
static DjiMotorInstance_s *motor_dji_instances[DJI_MOTOR_MAX_CNT]; // 存储电机实例的数组

/**
 * @file dev_motor_dji
 * @brief 大疆电机保护
 * @param output 电机输出
 * @param max 最大输出值
 * @return result--保护后的输出
 * @date 2025-08-18
 */
static float Motor_Dji_Protect(float output, float max) {
    float result;
    if(output > 0) {
        result = output > max ? max : output;
    }else if(output < 0) {
        result = output < -max ? -max : output;
    }
    return result;
}


/**
 * @file Motor_Dji
 * @brief 大疆电机分组函数
 * @return true--分组成功
 * @note 大疆电机可以一帧控制四个电机，所以需要对同一控制帧的电机进行分组
 * @date 2025-07-04
 */
#ifdef USER_CAN_STANDARD
static bool Motor_Dji_Grouping(uint32_t tx_id, CAN_HandleTypeDef *can_handle, uint8_t tx_buff[8]) {
#endif
#ifdef USER_CAN_FD
    static bool Motor_Dji_Grouping(uint32_t tx_id, FDCAN_HandleTypeDef *can_handle, uint8_t tx_buff[8]) {
#endif
    for(int i = 0; i < idx; i++) {
        if(motor_dji_instances[i]->can_instance->can_handle == can_handle && motor_dji_instances[i]->can_instance->tx_id == tx_id) {
           if(motor_dji_instances[i]->id < 5){   
                tx_buff[2*(motor_dji_instances[i]->id-1)] = ((int16_t)motor_dji_instances[i]->output>>8)&0xff;
                tx_buff[2*(motor_dji_instances[i]->id-1)+1] = (int16_t)motor_dji_instances[i]->output&0xff;
            }else{
                tx_buff[2*(motor_dji_instances[i]->id-5)] = ((int16_t)motor_dji_instances[i]->output>>8)&0xff;
                tx_buff[2*(motor_dji_instances[i]->id-5)+1] = (int16_t)motor_dji_instances[i]->output&0xff;
            }
        }
    }
    return true;
}

/**
 * @file Motor_Dji_Decode
 * @brief 大疆电机解码
 * @date 2025-07-04
 */
static void Motor_Dji_Decode(CanInstance_s *can_instance){
     if(can_instance == NULL){
        return;
    }
    DjiMotorInstance_s *motor = can_instance->parent_ptr; // 获取电机实例
    
    // 1. 更新转子位置记录
    if(motor->message.rotor_position != 0){
        motor->message.rotor_last_position = motor->message.rotor_position;
    }
    motor->message.rotor_position = (int16_t)(can_instance->rx_buff[0]<< 8 | can_instance->rx_buff[1]);
    motor->message.rotor_velocity = (int16_t)(can_instance->rx_buff[2] << 8 | can_instance->rx_buff[3]);
    motor->message.torque_current = (int16_t)(can_instance->rx_buff[4] << 8 | can_instance->rx_buff[5]);
    motor->message.temperature = can_instance->rx_buff[6];

    // 计算输出轴速度
    motor->message.out_velocity = motor->message.rotor_velocity / motor->reduction_ratio;

    // 2. 处理过零点，计算转子转过的差值 (delta)
    int32_t delta = 0; // 使用int32防止计算过程溢出
    int32_t diff = motor->message.rotor_position - motor->message.rotor_last_position;
    
    // 如果差值超过半圈(4096)，说明发生了过零点
    if (diff > 4096) {
        delta = diff - 8192;
    } else if (diff < -4096) {
        delta = diff + 8192;
    } else {
        delta = diff;
    }

    // 3. 累加总角度 (int64_t 不会溢出)
    motor->message.total_angle += delta;

    // 4. 核心修复：先转为总弧度，再归一化
    // 公式：总弧度 = (总编码器值 * 2PI/8192) / 减速比
    double total_rad = (double)motor->message.total_angle * DJI_ECD_ANGLE_COEF / motor->reduction_ratio;
    
    // 使用 fmod 对 2PI 取模，结果范围是 (-2PI, 2PI)
    motor->message.out_position = (float)fmod(total_rad, 2.0 * M_PI);

    // 5. 将范围调整到 (-PI, PI]
    if (motor->message.out_position > M_PI) {
        motor->message.out_position -= 2.0f * M_PI;
    } else if (motor->message.out_position <= -M_PI) {
        motor->message.out_position += 2.0f * M_PI;
    }
}


DjiMotorInstance_s *Motor_Dji_Register(DjiMotorInitConfig_s *config) {
    if (config == NULL || idx >= DJI_MOTOR_MAX_CNT) {
        return NULL;
    }
    // 分配内存
    DjiMotorInstance_s *motor_instance = (DjiMotorInstance_s *)user_malloc(sizeof(DjiMotorInstance_s));
    memset(motor_instance, 0, sizeof(DjiMotorInstance_s)); // 清空内存
    // 初始化电机基本信息
    motor_instance->topic_name = config->topic_name;
    motor_instance->type = config->type;
    motor_instance->control_mode = config->control_mode;
    motor_instance->reduction_ratio = config->reduction_ratio;
    if(config->reduction_ratio == 0){
        #ifdef USER_LOG
        Log_Error("where`s your fucking reduction ratio config?");
        #endif
        return NULL;
    }
    // 计算id
    if(config->type == GM6020){
        motor_instance->id = (config->can_config.rx_id - 4) % 0x200;
    }else{
        motor_instance->id = config->can_config.rx_id % 0x200;
    }

    // 初始化PID
    motor_instance->angle_pid = Pid_Register(&config->angle_pid_config);
    motor_instance->velocity_pid = Pid_Register(&config->velocity_pid_config);
    // 注册电机到CAN总线
    config->can_config.topic_name = config->topic_name;
    config->can_config.can_module_callback = Motor_Dji_Decode;
    config->can_config.parent_ptr = motor_instance;
    motor_instance->can_instance = Can_Register(&config->can_config);
    if (motor_instance->can_instance == NULL) {
        user_free(motor_instance); // 释放内存
        return NULL;
    }
    motor_dji_instances[idx++] = motor_instance; // 将电机实例添加到电机数组中
    return motor_instance;
}

bool Motor_Dji_Control(DjiMotorInstance_s *motor, float taget) {
    if(motor == NULL || motor->can_instance == NULL) {
        return false; // 无效的电机实例
    }
    if(motor->control_mode == DJI_POSITION){ // 位置控制模式
        motor->target_position = taget;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->message.out_position);
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
    }else if(motor->control_mode == DJI_VELOCITY){ // 速度控制模式
        motor->target_velocity = taget;
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
    }
    return true;
}

bool Motor_Dji_SetCurrent(DjiMotorInstance_s *motor, float set_current){
    if(motor == NULL) {
        return false;
    }
    switch(motor->type) {
        case GM6020:
            if(motor->can_instance->tx_id == 0x1FF || motor->can_instance->tx_id == 0x2FF) {
                motor->output = Motor_Dji_Protect(set_current, 25000);
            }else{
                motor->output = Motor_Dji_Protect(set_current, 16384);
            }
            break;
        case M3508:
            motor->output = Motor_Dji_Protect(set_current, 16384);
            break;
        case M2006:
            motor->output = Motor_Dji_Protect(set_current, 10000);
            break;
    }
    return true;
}


bool Motor_Dji_Transmit(DjiMotorInstance_s *motor) {
    if(motor == NULL || motor->can_instance == NULL) {
        return false;
    }
    Motor_Dji_Grouping(motor->can_instance->tx_id, motor->can_instance->can_handle, motor->can_instance->tx_buff);
    if(Can_Transmit(motor->can_instance) == true){
        return true;
    } else {
        return false;
    }
}

bool Motor_Dji_Change_Mode(DjiMotorInstance_s *motor, DjiMotorControlMode_e target_mode){
    if(motor == NULL) {
        return false;
    }
    motor->control_mode = target_mode;
    return true;
}



