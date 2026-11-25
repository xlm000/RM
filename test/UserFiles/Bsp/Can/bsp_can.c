/**
* @file bsp_can.c
 * @author He WenXuan (hewenxuan040923@gmail.com)
 * @brief CAN驱动模块
 * @version 0.1
 * @details CAN驱动模块,提供CAN的初始化、发送、接收等功能
 * @date 2025-07-04
 * @update 2025-08-26
 *       1. 增加对fifo0和fifo1的支持，用户可根据需要选择使用哪个fifo，而非指定使用fifo0或fifo1、
 *       2. 完成之前的to do
 *          @done 1. 与HAL库和STM32F4彻底解耦，提供更灵活的接口
 *          @done 2. 支持更多的报错信息提醒，方便调试
 *          @done 3. 增加对CAN FD的支持 //对两个文件进行了分离
 * @copyright  Copyright (c) 2025 HDU—PHOENIX
 * @todo 1.Can_Transmit函数中增加发送超时机制,防止死等待,以及对HAL库函数的重写，提升性能
 */


#include "bsp_can.h"
#ifdef USER_CAN_STANDARD
#ifdef USER_LOG
#include "bsp_log.h"
#endif
#include <string.h>
#include <stdbool.h>

bool can_init_flag = false;// CAN模块初始化标志，防止重复初始化

#ifdef USER_CAN1
static uint8_t can_idx1;
static CanInstance_s *can1_instance[CAN_MAX_REGISTER_CNT]; // CAN1 实例数组,
#endif
#ifdef USER_CAN2
static uint8_t can_idx2;
static CanInstance_s *can2_instance[CAN_MAX_REGISTER_CNT]; // CAN2 实例数组
#endif

static uint8_t can_filter_index = 0;


static CAN_RxFrame_TypeDef CAN_RxFIFO0Frame;
static CAN_RxFrame_TypeDef CAN_RxFIFO1Frame;
static uint8_t can_fifo_select_flag = 0; // 0表示使用fifo0,1表示使用fifo1

/**
 * @brief 初始化CAN模块。
 * @todo 需要添加超时警告机制，防止初始化过程中的死循环,但是能做到初始化失败的也是神人了
 */
bool Can_Init(void) {

    // 配置 CAN1_FIFO_0 过滤器
    CAN_FilterTypeDef filter_can1_config;
    filter_can1_config.FilterActivation = CAN_FILTER_ENABLE;
    filter_can1_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_can1_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_can1_config.FilterIdHigh = 0x0000;
    filter_can1_config.FilterIdLow = 0x0000;
    filter_can1_config.FilterMaskIdHigh = 0x0000;
    filter_can1_config.FilterMaskIdLow = 0x0000;
    filter_can1_config.FilterBank = 0;
#ifdef USER_CAN1_FIFO_0
    filter_can1_config.FilterFIFOAssignment = CAN_RX_FIFO0;
#endif
#ifdef USER_CAN1_FIFO_1
    filter_can1_config.FilterFIFOAssignment = CAN_RX_FIFO1;
#endif
    if (HAL_CAN_ConfigFilter(&hcan1, &filter_can1_config) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN1 FIFO0 Filter Config Error");
        #endif
        return false;
    }
#ifdef USER_CAN2
    CAN_FilterTypeDef filter_can2_config;
    filter_can2_config.FilterActivation = CAN_FILTER_ENABLE;
    filter_can2_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_can2_config.FilterScale = CAN_FILTERSCALE_32BIT;
#ifdef USER_CAN2_FIFO_0
    filter_can2_config.FilterFIFOAssignment = CAN_RX_FIFO0;
#endif
#ifdef USER_CAN2_FIFO_1
    filter_can2_config.FilterFIFOAssignment = CAN_RX_FIFO1;
#endif
    filter_can2_config.FilterIdHigh = 0x0000;
    filter_can2_config.FilterIdLow = 0x0000;
    filter_can2_config.FilterMaskIdHigh = 0x0000;
    filter_can2_config.FilterMaskIdLow = 0x0000;
    filter_can2_config.FilterBank = 14;
    filter_can2_config.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &filter_can2_config) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN2 FIFO0 Filter Config Error");
        #endif
        return false;
    }
#endif
    #ifdef USER_LOG
    Log_Passing("Can MASK Filter Init successfully");
    #endif
#endif
    #ifdef USER_CAN1
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN1 Starts Failed");
        #endif
        return false;
    }
#ifdef USER_CAN2
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN2 Starts Failed");
        #endif
        return false;
    }
#endif
#endif
#ifdef USER_CAN1_FIFO_0
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN1 FIFO0 Interruption Config Error");
        #endif
        return false;
    }
#endif
#ifdef USER_CAN2_FIFO_0
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN2 FIFO0 Interruption Config Error");
        #endif
        return false;
    }
#endif

#ifdef USER_CAN1_FIFO_1
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN1 FIFO1 Interruption Config Error");
        #endif
        return false;
    }
#endif
#ifdef USER_CAN2_FIFO_1
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        #ifdef USER_LOG
        Log_Error("CAN2 FIFO1 Interruption Config Error");
        #endif
        return false;
    }
#endif
    can_init_flag = true; // 初始化完成
    #ifdef USER_LOG
    Log_Passing("Can Init successfully");
    #endif
    return true;
}


/**
 * @brief 根据CAN编号选择对应的CAN句柄,主要是与FDCAN解耦
 * @param can_number CAN编号，1表示CAN1，2表示CAN2。
 * @return 指向对应CAN句柄的指针，如果编号无效则返回NULL。
 */
static CAN_HandleTypeDef *CAN_Select_Handle(const uint8_t can_number) {
#ifdef USER_CAN1
    if (can_number == 1) {
        return &hcan1;
    }
#endif
#ifdef USER_CAN2
    if (can_number == 2) {
        return &hcan2;
    }
#endif
    return NULL; //防止编译警告,我无法想象有人在没开启can的情况下能调用这个函数
}

/**
 * @brief 将CAN实例放到到对应的CAN实例数组中。
 * @param instance 指向要注册的CanInstance_s结构体的指针
 */
static void Can_Register_To_x_Instance(CanInstance_s *instance) {
#ifdef USER_CAN1
    if (instance->can_handle == &hcan1) {
        can1_instance[can_idx1] = instance;
        can_idx1++;
        #ifdef USER_LOG
        Log_Passing("%s : Can1 Register Successfully, Tx ID:0x%03X, Rx ID:0x%03X", instance->topic_name,
                    instance->tx_id,
                    instance->rx_id);
        #endif
    }   
#ifdef USER_CAN2
    if (instance->can_handle == &hcan2) {
        can2_instance[can_idx2] = instance;
        can_idx2++;
        #ifdef USER_LOG
        Log_Passing("%s : Can2 Register Successfully, Tx ID:0x%03X, Rx ID:0x%03X", instance->topic_name,
                    instance->tx_id,
                    instance->rx_id);
        #endif
    }
#endif
#endif
}

CanInstance_s *Can_Register(CanInitConfig_s *can_config) {
    // 如果是第一次注册CAN实例，则初始化CAN模块
    while (can_init_flag == false) {
        Can_Init();
    }

    // 分配内存并初始化CAN实例
    CanInstance_s *instance = user_malloc(sizeof(CanInstance_s));
    if (instance == NULL) {
        #ifdef USER_LOG
        Log_Error("%s : Can Register Failed, No Memory", can_config->topic_name);
        #endif
        return NULL;
    }
    // 清空内存
    memset(instance, 0, sizeof(CanInstance_s));
    instance->topic_name = can_config->topic_name;
    instance->can_handle = CAN_Select_Handle(can_config->can_number);
    instance->tx_id = can_config->tx_id;
    instance->tx_header = (CAN_TxHeaderTypeDef){
        .StdId = can_config->tx_id,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 0x08,
        .TransmitGlobalTime = DISABLE
    };
    instance->rx_id = can_config->rx_id;
    instance->can_module_callback = can_config->can_module_callback;
    instance->parent_ptr = can_config->parent_ptr;
    // 将实例注册到对应的CAN实例数组中
    Can_Register_To_x_Instance(instance);
    return instance;
}
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff) {
    /* 检查实例和发送缓冲区是否有效 */
    if (instance == NULL || tx_buff == NULL) {
        #ifdef USER_LOG
        Log_Error("Can Transmit Failed, Instance or Tx Buff is NULL");
        #endif
        return false;
    }
	uint8_t can_tx_cnt = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0) {
		can_tx_cnt++;
		if(can_tx_cnt>100){ //大概10ms
            #ifdef USER_LOG
            Log_Error("Can Transmit Failed, No Mailbox Free");
            #endif
			return false;
		}
    }
    if (HAL_CAN_AddTxMessage(instance->can_handle, &instance->tx_header, (uint8_t *) tx_buff,
                             (uint32_t *) CAN_TX_MAILBOX0) == HAL_OK) {
        return true;
    }
    return false;
}

bool Can_Transmit(CanInstance_s *instance) {
	if(instance==NULL) return false;
	uint8_t can_tx_cnt = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0) {
		can_tx_cnt++;
		if(can_tx_cnt>100){
            #ifdef USER_LOG
            Log_Error("Can Transmit Failed, No Mailbox Free");
            #endif
			return false;
		}
    }
    uint32_t tx_mailbox = CAN_TX_MAILBOX0;
    instance->tx_header.StdId = instance->tx_id; //确保发送ID正确
    if (HAL_CAN_AddTxMessage(instance->can_handle, &instance->tx_header, (uint8_t *) instance->tx_buff,
                             &tx_mailbox) == HAL_OK) {
        return true;
    }
    return false;
}

/**
 * @brief 处理CAN接收FIFO中的消息。
 * @param CAN_RxFIFOxFrame 指向包含接收到的CAN消息的CAN_RxFrame_TypeDef结构的指针
 * @param idx 已注册的CAN实例数量
 * @param can_instance 指向已注册的CanInstance_s结构体数组的指针
 */
static void USER_CAN_RxFifo0MsgPendingCallback(CAN_RxFrame_TypeDef *CAN_RxFIFOxFrame, const uint8_t idx,
                                               CanInstance_s **can_instance) {
    for (uint8_t i = 0; i < idx; i++) {
        if (CAN_RxFIFOxFrame->RxHeader.StdId == can_instance[i]->rx_id) {
            can_instance[i]->rx_len = CAN_RxFIFOxFrame->RxHeader.DLC;
            memcpy(can_instance[i]->rx_buff, CAN_RxFIFOxFrame->rx_buff, can_instance[i]->rx_len);
            can_instance[i]->cnt++; // 通信计数+1
            if(can_instance[i]->cnt>=0xFFFF) can_instance[i]->cnt=0; //防止溢出
            // 调用用户定义的回调函数
            if (can_instance[i]->can_module_callback != NULL) {
                can_instance[i]->can_module_callback(can_instance[i]);
            }
            break;
        }
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_fifo_0_msg_count = HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO0);
    if (rx_fifo_0_msg_count >= 7) {
       #ifdef USER_LOG
        Log_Warning("RX FIFO0 Message Count is %d", rx_fifo_0_msg_count);
        #endif
    }
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxFIFO0Frame.RxHeader, CAN_RxFIFO0Frame.rx_buff);
#ifdef USER_CAN1
    if (hcan == &hcan1) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO0Frame, can_idx1, can1_instance);
    }
#ifdef USER_CAN2
    if (hcan == &hcan2) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO0Frame, can_idx2, can2_instance);
    }
#endif
#endif
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_fifo_1_msg_count = HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO1);
    if (rx_fifo_1_msg_count >= 7) {
        #ifdef USER_LOG
        Log_Warning("RX FIFO1 Message Count is %d", rx_fifo_1_msg_count);
        #endif
    }

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_RxFIFO1Frame.RxHeader, CAN_RxFIFO1Frame.rx_buff);
#ifdef USER_CAN1
    if (hcan == &hcan1) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO1Frame, can_idx1, can1_instance);
    }
#ifdef USER_CAN2
    if (hcan == &hcan2) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO1Frame, can_idx2, can2_instance);
    }
#endif
#endif
}

