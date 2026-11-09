//
// Created by 朱泽超 on 2026/11/8.
//

#include "GimbalTask.h"
#include "Gimbal_fun.h"

#include "dev_dr16.h"
#include "dev_motor_dm.h"
#include "dev_motor_rs.h"

#include "cmsis_os.h"
#include "usb_device.h"

Dr16Instance_s *dr16_instance;

void GimbalTask(void const * argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN GimbalTask */

    //实例注册区
    dr16_instance = Dr16_Register(&huart6);

    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END GimbalTask */
}