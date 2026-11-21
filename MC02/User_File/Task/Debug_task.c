#include "Debug_task.h"
#include "drv_usart.h"
#include "dvc_stepmotor.h"
#include "dvc_hmi.h"
#include "dvc_yis320.h"
#include "dvc_remote.h"
#include "dvc_vofa.h"
#include "IMU_task.h"
#include "Chassis_task.h"
#include "Chassis_Odometry.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"
#include "usart.h"
#include  "drv_can.h"
#include <stdio.h>


void Debug_Init(void)
{
    HMI_Init();
    SBUS_Init(Remote_Callback);

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

            /* 5/6 号（升降/伸缩）也轮询实时位置：没有调试器，只能靠屏上的绝对值
               区分"指令没发出去"和"已经在目标位置上"。每轮发一帧读请求，
               先看 TX FIFO 有没有空位，别和底盘的命令抢那 3 个槽位。 */
            static const uint8_t k_tower_addr[2] = { 5u, 6u };
            static uint8_t tower_idx = 0u;
            if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0u)
            {
                Emm_V5_Read_Sys_Params(k_tower_addr[tower_idx], S_CPOS);
                tower_idx ^= 1u;
            }

            /* chassis_pos/chassis_ang 是 2 位小数控件，Send_Float 内部 ×100，屏上读数就是 mm */
            HMI_Send_Float("chassis_pos.val", chassis.Px);   /* Px, mm */
            HMI_Send_Float("chassis_ang.val", IMU_getYaw());   /* Py, mm */
            HMI_Send_Float("yaw_float.val",  IMU_getYaw());     /* C2 */

            HMI_Send_Number("motor01.val", (int)Emm_GetPosRaw(1));     /* C1 */
            HMI_Send_Number("motor02.val", (int)Emm_GetPosRaw(2));     /* C2 */
            HMI_Send_Number("motor03.val", (int)Emm_GetPosRaw(3));     /* C3 */
            HMI_Send_Number("motor04.val", (int)Emm_GetPosRaw(4));     /* C4 */

             HMI_Send_Float("pos_x.val",  chassis.Px);
             HMI_Send_Float("pos_y.val",  chassis.Py);

            /* R=已发读请求 / 收到帧 / 丢弃帧 / TX 入队失败，最近一帧 功能码:长度，
               L=1s 内有应答的电机位图（bit0~bit5 = 1~6 号），P=5/6 号实时绝对位置，
               B=bus-off 次数（上电时电机没起来会 +1，恢复后总线自动重启） */
            uint32_t now  = HAL_GetTick();
            unsigned live = 0u;
            for (uint8_t i = 1u; i <= 6u; i++)
            {
                if (Emm_Feedback[i].valid &&
                    (uint32_t)(now - Emm_Feedback[i].tick) < 1000u)
                {
                    live |= (1u << (i - 1u));
                }
            }

            char st[96];
            snprintf(st, sizeof(st), "R%lu/%lu/%lu/%lu %02X:%u L%X P%ld/%ld B%lu",
                    (unsigned long)Emm_ReadReqTotal,
                    (unsigned long)Emm_CanRxAll,
                    (unsigned long)Emm_CanRxDrop,
                    (unsigned long)can_tx_fail_cnt,
                    (unsigned int)Emm_LastFrameData[0],
                    (unsigned int)Emm_LastFrameDlc,
                    live,
                    (long)Emm_GetPosRaw(5),
                    (long)Emm_GetPosRaw(6),
                    (unsigned long)can_busoff_cnt);
            HMI_Send_String("name_yaw.txt", st);

            /* 遥控器数据走 USB 虚拟串口（vofa float 帧），任务上下文发送 */
            vofa_send_remote();
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

void Remote_Callback()
{
    /* 遥控器数据解码后，直接赋值给底盘速度 */
    chassis.setVx = (float)(Remote_control_FS.Right_Y - 1024) * 0.1f; // mm/s
    chassis.setVy = (float)(Remote_control_FS.Right_X - 1024) * 0.1f; // mm/s
    chassis.setVw = (float)(Remote_control_FS.Left_X - 1024) * 0.001f; // rad/s
}