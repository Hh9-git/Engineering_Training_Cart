#include "Debug_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"

/* 彩色渐变：色相每帧推进 2 度，10ms 一帧 -> 1.8s 走完一圈 */
#define WS2812_HUE_STEP     2U
#define WS2812_FRAME_MS     10U
#define WS2812_SAT          255U
#define WS2812_VAL          150U

void Debug_Init(void)
{
    // Initialization code for the Debug task can be added here

    AttachInterrupt_FDCAN(&hfdcan1,ZDT_CAN_Callback);

	FDCAN_FilterInit_Ext(&hfdcan1);                    // 覆盖为扩展帧全通


}

void Debug_Task(void *argument)
{
    /* BASEPRI 已在 StartDefaultTask 里清过（内核级全局寄存器），这里再清一次
     * 是防御性写法 —— 本任务的栈和调度顺序将来都可能变。 */
    __set_BASEPRI(0U);

    Debug_Init();

    uint16_t hue = 0;

    // Main loop for the Debug task
    while (1)
    {
        WS2812_CtrlHSV(hue, WS2812_SAT, WS2812_VAL);
        hue = (uint16_t)((hue + WS2812_HUE_STEP) % 360U);
        osDelay(WS2812_FRAME_MS); // Delay to prevent busy waiting
    }
}