#include "Debug_task.h"
#include "drv_usart.h"
#include "dvc_stepmotor.h"
#include "dvc_hmi.h"
#include "IMU_task.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "usart.h"


void Debug_Init(void)
{
    HMI_Init();                 
}

void Debug_Task(void *argument)
{
    Debug_Init();
    for (;;)
    {
       HMI_Send_String("name_yaw.txt","hello");
        HMI_Send_Float("yaw_float.val", imuAngle[INS_YAW_ADDRESS_OFFSET] * 57.29578f);
          osDelay(10);
        HMI_Send_Float("temp.val", imuTemp);

        osDelay(10);
    }
}