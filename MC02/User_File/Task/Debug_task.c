#include "Debug_task.h"
#include "drv_usart.h"
#include "dvc_stepmotor.h"
#include "dvc_hmi.h"
#include "dvc_yis320.h"
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
    uint32_t last_wd = HAL_GetTick();

    Debug_Init();
    for (;;)
    {
       HMI_Send_String("name_yaw.txt","hello");
        HMI_Send_Float("yaw_float.val", IMU_getYaw());   /* YIS320 的 yaw，单位已经是度 */


        //  HMI_Send_Float("yaw_float.val", imuAngle[INS_YAW_ADDRESS_OFFSET] * RAD_TO_DEG);
          osDelay(10);
        HMI_Send_Float("temp.val", imuTemp);

        osDelay(10);

        /* YIS320 接收被 ORE 停掉时补挂回去，否则线上抖一下就会永久收不到 */
        if (HAL_GetTick() - last_wd >= 500u)
        {
            last_wd = HAL_GetTick();
            IMU_RxWatchdog();
        }
    }
}