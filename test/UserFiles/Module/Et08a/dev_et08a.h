#ifndef __DEV_ET08A_H__
#define __DEV_ET08A_H__

#include "bsp_uart.h"
#include "robot_config.h"
#include <stdint.h>

typedef struct {
    UartInstance_s *uart_instance;      // UART实例指针
    //摇杆
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    //拨杆开关
    int8_t sa;
    int8_t sb;
    int8_t sc;
    int8_t sd;

    int16_t ld;
    int16_t rd;

    int16_t t2;
    int16_t t3;
}Et08aInstance_s;

Et08aInstance_s* Et08a_Register(UART_HandleTypeDef *husart);

#endif
