#include "dev_et08a.h"
#include <string.h>
#include <stdlib.h>


static void Et08a_Decode(UartInstance_s* uart_instance){
    if(uart_instance == NULL || uart_instance->id == NULL) return;
    Et08aInstance_s* et08a = (Et08aInstance_s *)uart_instance->id; // 获取实例指针
    
     if(uart_instance->rx_size != 25){
        memset(et08a->uart_instance->rx_buff,0,sizeof(et08a->uart_instance->rx_buff));
        memset(&et08a->lx,0,sizeof(et08a->lx));
        memset(&et08a->ly,0,sizeof(et08a->ly));
        memset(&et08a->rx,0,sizeof(et08a->rx));
        memset(&et08a->ry,0,sizeof(et08a->ry));
        memset(&et08a->sa,0,sizeof(et08a->sa));
        memset(&et08a->sb,0,sizeof(et08a->sb));
        memset(&et08a->sc,0,sizeof(et08a->sc));
        memset(&et08a->sd,0,sizeof(et08a->sd));
    } else {
        et08a->rx = uart_instance->rx_buff[1] | ((uart_instance->rx_buff[2] & 0x07) << 8) - 1024;
        et08a->ry = (((uart_instance->rx_buff[2] & 0xf8) >> 3) | ((uart_instance->rx_buff[3] & 0x3f) << 5)) - 1024;
        et08a->ly = (((uart_instance->rx_buff[3] & 0xc0) >> 6) | ((uart_instance->rx_buff[4] & 0xff ) << 2) | ((uart_instance->rx_buff[5] & 0x01) << 10)) - 1024;
        et08a->lx = (((uart_instance->rx_buff[5] & 0xfe) >> 1) | ((uart_instance->rx_buff[6] & 0x0f)) << 7) - 1024;

        uint16_t s_temp[4];
        s_temp[0] = ((uart_instance->rx_buff[6] & 0xf0) >> 4) | ((uart_instance->rx_buff[7] & 0x7f)) << 4;
        s_temp[1] = ((uart_instance->rx_buff[7] & 0x80) >> 7) | (uart_instance->rx_buff[8] << 1) | ((uart_instance->rx_buff[9] & 0x03) << 9);
        s_temp[2] = ((uart_instance->rx_buff[9] & 0xfc) >> 2) | ((uart_instance->rx_buff[10] & 0x1f) << 6);
        s_temp[3] = ((uart_instance->rx_buff[10] & 0xe0) >> 5) | (uart_instance->rx_buff[11] << 3);
        if (s_temp[0] >= 1400 && s_temp[0] <= 3000){
            et08a->sa = 2;
            et08a->ld = s_temp[0] - 1694;
        }else if (s_temp[0] >= 100 && s_temp[0] <= 600){
            et08a->sa = 1;
            et08a->ld = s_temp[0] - 353;
        }

        if (s_temp[1] >= 800 && s_temp[1] <= 1300){
            et08a->sb = 3;
            et08a->t2 = 1024 - s_temp[1];
        }else if (s_temp[1] >= 1400 && s_temp[1] <= 3000){
            et08a->sb = 2;
            et08a->t2 = 1694 - s_temp[1];
        }else if (s_temp[1] >= 100 && s_temp[1] <= 600){
            et08a->sb = 1;
            et08a->t2 = 353 - s_temp[1];
        }

        if (s_temp[2] >= 800 && s_temp[2] <= 1300){
            et08a->sc = 3;
            et08a->t3 = 1024 - s_temp[2] ;
        }else if (s_temp[2] >= 1400 && s_temp[2] <= 3000 ){
            et08a->sc = 2;
            et08a->t3 = 1694 - s_temp[2];
        }else if (s_temp[2] >= 100 && s_temp[2] <= 600){
            et08a->sc = 1;
            et08a->t3 = 353 - s_temp[2];
        }

        if (s_temp[3] >= 1400 && s_temp[3] <= 3000){
            et08a->sd = 2;
            et08a->rd = s_temp[3] - 1694;
        }else if (s_temp[3] >=100 && s_temp[3] <= 600){
            et08a->sd = 1;
            et08a->rd = s_temp[3] - 353;
        }
        // 保护，大于675都认为是无效值(实际应该在671以内，但是由于经常飘，故给5的余量)
       // if (et08a->ld >100)et08a->sb = 1;
       //  else if (et08a->ld <-100) et08a->sb = 2;
       //  else et08a->sb = 3;
        et08a->lx = (abs(et08a->lx) > 675) ? 0 : et08a->lx;
        et08a->ly = (abs(et08a->ly) > 675) ? 0 : et08a->ly;
        et08a->rx = (abs(et08a->rx) > 675) ? 0 : et08a->rx;
        et08a->ry = (abs(et08a->ry) > 675) ? 0 : et08a->ry;
    }
    
    HAL_UARTEx_ReceiveToIdle_DMA(et08a->uart_instance->uart_handle,
                             et08a->uart_instance->rx_buff,
                             sizeof(et08a->uart_instance->rx_buff));
}

Et08aInstance_s* Et08a_Register(UART_HandleTypeDef *husart) {
    // 注册UART实例
    Et08aInstance_s *instance = (Et08aInstance_s *)user_malloc(sizeof(Et08aInstance_s));
    if (instance == NULL) {
        return NULL; // 内存分配失败
    }
    memset(instance, 0, sizeof(Et08aInstance_s)); // 清空内存

    UartInitConfig_s uart_config;
    // 配置UART实例
    uart_config.uart_handle = husart;
    uart_config.mode = UART_IDLE_MODE;
    uart_config.rx_len = 25;
    uart_config.uart_module_callback = Et08a_Decode;
    uart_config.id = instance;

    instance->uart_instance = Uart_Register(&uart_config); // 注册UART实例

    if (instance->uart_instance == NULL) {
        user_free(instance);
        return NULL;
    }
    return instance;
}


