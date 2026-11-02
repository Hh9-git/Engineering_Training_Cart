#include "Debug_task.h"
#include "drv_usart.h"
#include "dvc_stepmotor.h"
#include "dvc_hmi.h"
#include "dvc_yis320.h"
#include "IMU_task.h"
#include "Chassis_task.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "usart.h"
#include <stdio.h>


void Debug_Init(void)
{
    HMI_Init();
}

void Debug_Task(void *argument)
{
    uint32_t last_wd  = HAL_GetTick();
    uint32_t last_hmi = 0;

    Debug_Init();
    for (;;)
    {
        /* 串口屏 10Hz 刷新足够。原始计数是整数，必须用 Send_Number：
           Send_Float 会先 ×100，只适合屏幕按 2 位小数显示的控件。 */
        if (HAL_GetTick() - last_hmi >= 100u)
        {
            last_hmi = HAL_GetTick();

            /* chassis_pos/chassis_ang 是 2 位小数控件，Send_Float 内部 ×100，屏上读数就是 mm */
            HMI_Send_Float("chassis_pos.val", chassis.Px);   /* Px, mm */
            HMI_Send_Float("chassis_ang.val", IMU_getYaw());   /* Py, mm */
            HMI_Send_Float("yaw_float.val",  IMU_getYaw());     /* C2 */

            HMI_Send_Float("motor01.val",  (float)Emm_GetPosRaw(1));     /* C1 */
            HMI_Send_Float("motor02.val",  (float)Emm_GetPosRaw(2));     /* C2 */
            HMI_Send_Float("motor03.val",  (float)Emm_GetPosRaw(3));     /* C3 */
            HMI_Send_Float("motor04.val",  (float)Emm_GetPosRaw(4));     /* C4 */

             HMI_Send_Float("pos_x.val",  chassis.Px);    
             HMI_Send_Float("pos_y.val",  chassis.Py);

            /* 收到帧 / 丢弃 / 发送失败 + 最近一帧 功能码:长度 */
            char st[32];
            sprintf(st, "%lu/%lu/%lu %02X:%u",
                    (unsigned long)Emm_CanRxAll,
                    (unsigned long)Emm_CanRxDrop,
                    (unsigned long)can_tx_fail_cnt,
                    (unsigned int)Emm_LastFrameData[0],
                    (unsigned int)Emm_LastFrameDlc);
            HMI_Send_String("name_yaw.txt", st);
        }

        osDelay(10);

        /* YIS320 接收被 ORE 停掉时补挂回去，否则线上抖一下就会永久收不到 */
        if (HAL_GetTick() - last_wd >= 500u)
        {
            last_wd = HAL_GetTick();
            IMU_RxWatchdog();
        }
    }
}