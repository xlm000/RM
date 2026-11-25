/**
* @file bsp_uart.c
 * @author Wenxin HU
 * @brief UART驱动模块
 * @version 0.1
 * @details UART,提供UART的初始化、发送、接收等功能
 * @date 2025-07-11
 * @copyright  Copyright (c) 2025 HDU—PHOENIX
 * @todo
 *       1. 阻塞模式更好的实现方式
 */

#include "bsp_uart.h"
#include "usart.h"
#include "robot_config.h"
#include <string.h>


//串口并非总线，是一对一的形式，所以一个实例对应一个物理串口，并非CAN的一个物理CAN对应多个CAN示例
static uint8_t id = 0; // 用于标识UART实例的唯一id,从0开始
static UartInstance_s *uart_instance[UART_MAX_CNT]; // UART实例数组,用于存储UART注册的实例

/**
 * @brief 重定向的串口回调函数
 * @param huart 接收串口中断的串口句柄
 */
static void Uart_RxCallback(UART_HandleTypeDef *huart) {
    static UART_HandleTypeDef* _uart_handle;
    if (huart == NULL) {
        return;
    }
    for (uint8_t i = 0; i < id; i++) {
        if (uart_instance[i]->uart_handle == huart) { // 找到对应的UART实例
            if (uart_instance[i]->uart_module_callback != NULL) {
                uart_instance[i]->uart_module_callback(uart_instance[i]); // 调用回调函数处理接收到的数据
                uart_instance[i]->rx_cnt++; // 通信计数+1
                if(uart_instance[i]->rx_cnt>=0xFFFF) uart_instance[i]->rx_cnt=0; //防止溢出
            }
            break; // 找到后退出循环
        }
    }
}

static void Uart_RxIDLECallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == NULL) return;

    for (uint8_t i = 0; i < id; i++) {
        if (uart_instance[i]->uart_handle == huart) {
            uart_instance[i]->rx_size = Size;

            if (uart_instance[i]->mode == UART_IDLE_MODE &&
                uart_instance[i]->uart_module_callback != NULL) {
                uart_instance[i]->uart_module_callback(uart_instance[i]);

                uart_instance[i]->rx_cnt++;
                if (uart_instance[i]->rx_cnt >= 0xFFFF) {
                    uart_instance[i]->rx_cnt = 0;
                }
                }
            break;
        }
    }
}

static void Uart_TxCallback(UART_HandleTypeDef *huart) {
    if(huart == NULL) {
        return;
    }
    for(uint8_t i = 0; i < id; i++) {
        if(uart_instance[i]->uart_handle == huart) {
            if(uart_instance[i]->uart_module_tx_callback != NULL) {
                uart_instance[i]->uart_module_tx_callback(uart_instance[i]);
            }
            break;
        }
    }
}

UartInstance_s *Uart_Register(UartInitConfig_s *config) {
    if (config == NULL || id >= UART_MAX_CNT) {
        return NULL;//如果空间已满或者没有配置信息，返回NULL
    }

    //检验注册的串口是否合法（在开启的范围内）
    if (config->uart_handle == NULL) {
        return NULL;
    }

    for (uint8_t i = 0; i < id ; i++) {
        if (uart_instance[i]->uart_handle == config->uart_handle) {
            return NULL; // 如果已经注册过了，返回NULL
        }
    }

    //开始分配空间 1用来区分实例和数组
    UartInstance_s *uart_instance1 = (UartInstance_s *)user_malloc(sizeof(UartInstance_s));
    memset(uart_instance1, 0, sizeof(UartInstance_s));//清空空间

    uart_instance1->uart_handle = config->uart_handle; // 设置串口句柄
    uart_instance1->mode = config->mode; // 设置串口通讯模式
    uart_instance1->rx_len = config->rx_len; // 设置接收长度
    uart_instance1->id = config->id;
    uart_instance1->uart_module_callback = config->uart_module_callback; // 设置回调函数
    uart_instance1->uart_module_tx_callback = config->uart_module_tx_callback; // 设置发送完成回调函数

    if (uart_instance1 == NULL || config->rx_len > UART_RX_BUFF_LEN) {
        user_free(uart_instance1); // 分配失败，释放内存
        return NULL;
    }

    uart_instance[id++] = uart_instance1; // 将实例添加到UART实例数组中

    if (uart_instance1->mode == UART_IT_MODE) {
        HAL_UART_Receive_IT(uart_instance1->uart_handle, uart_instance1->rx_buff, uart_instance1->rx_len); // 启用中断接收
    }else if (uart_instance1->mode == UART_DMA_MODE) {
        HAL_UART_Receive_DMA(uart_instance1->uart_handle, uart_instance1->rx_buff, uart_instance1->rx_len);
    }else if (uart_instance1->mode == UART_IDLE_MODE) {
        HAL_UARTEx_ReceiveToIdle_DMA(uart_instance1->uart_handle,
                                     uart_instance1->rx_buff,
                                     sizeof(uart_instance1->rx_buff));
        __HAL_DMA_DISABLE_IT(uart_instance1->uart_handle->hdmarx, DMA_IT_HT); // 禁用半满中断
    }

    return uart_instance1;
}

bool Uart_Transmit(UartInstance_s *uart_instance, uint8_t *data) {
    if (uart_instance == NULL || data == NULL) {
        return false; // 无效的参数
    }
    memset(&uart_instance->tx_buff,0,sizeof(uart_instance->tx_buff));

    for (size_t i = 0; i < UART_RX_BUFF_LEN; ++i) {
        if (data[i] == '\n') {
            uart_instance->tx_len = i + 1;
            break;
        }
    }

    memcpy(uart_instance->tx_buff, data, uart_instance->tx_len);

    if (uart_instance->mode == UART_BLOCKING_MODE) {
        HAL_UART_Transmit(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len, HAL_MAX_DELAY);
    } else if (uart_instance->mode == UART_IT_MODE) {
        HAL_UART_Transmit_IT(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else if (uart_instance->mode == UART_DMA_MODE) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else if (uart_instance->mode == UART_IDLE_MODE) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else {
        return false; // 不支持的模式
    }

    uart_instance->tx_cnt++; // 通信计数+1
    if(uart_instance->tx_cnt>=0xFFFF) uart_instance->tx_cnt=0; //防止溢出
    return true; // 发送成功
}

bool Uart_Transmit_Len(UartInstance_s* uart_instance, uint8_t* data, uint16_t len){
    if (uart_instance == NULL || data == NULL || len == 0 || len > 256) {
        return false; // 无效的参数
    }
    memset(&uart_instance->tx_buff,0,sizeof(uart_instance->tx_buff));
    uart_instance->tx_len = len;

    memcpy(uart_instance->tx_buff, data, uart_instance->tx_len);

    if (uart_instance->mode == UART_BLOCKING_MODE) {
        HAL_UART_Transmit(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len, HAL_MAX_DELAY);
    } else if (uart_instance->mode == UART_IT_MODE) {
        HAL_UART_Transmit_IT(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else if (uart_instance->mode == UART_DMA_MODE) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else if (uart_instance->mode == UART_IDLE_MODE) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else {
        return false; // 不支持的模式
    }

    uart_instance->tx_cnt++; // 通信计数+1
    if(uart_instance->tx_cnt>=0xFFFF) uart_instance->tx_cnt=0; //防止溢出
    return true; // 发送成功
}

bool Uart_Blocking_Receive(UartInstance_s* uart_instance) {
    if (uart_instance->mode != UART_BLOCKING_MODE) {
        return false; // 如果不是阻塞模式，返回false
    }

    if (HAL_UART_Receive(uart_instance->uart_handle, uart_instance->rx_buff, uart_instance->rx_len, HAL_MAX_DELAY) == HAL_OK ) {
        return true;
    }else {
        return false; // 接收失败
    }
}



//将HAL库的接收回调重定向到库的接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    Uart_RxCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    Uart_TxCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Uart_RxIDLECallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (uint8_t i = 0; i < id; i++) {
            if (uart_instance[i]->uart_handle == huart) {

            if (uart_instance[i]->mode == UART_IDLE_MODE) {
                HAL_UARTEx_ReceiveToIdle_DMA(huart, 
                                             uart_instance[i]->rx_buff, 
                                             UART_RX_BUFF_LEN);
                __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            } else if (uart_instance[i]->mode == UART_DMA_MODE) {
                HAL_UART_Receive_DMA(huart, 
                                     uart_instance[i]->rx_buff, 
                                     uart_instance[i]->rx_len);
            }
            break;
        }
    }
}