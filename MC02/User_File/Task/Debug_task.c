#include "Debug_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"

/* 彩色渐变：色相每帧推进 2 度，10ms 一帧 -> 1.8s 走完一圈 */
#define WS2812_HUE_STEP     2U
#define WS2812_FRAME_MS     10U
#define WS2812_SAT          255U
#define WS2812_VAL          150U

void Debug_Init(void)
{
    
   
}

void Debug_Task(void *argument)
{
    Debug_Init();
    uint16_t hue = 0;

     /* 启动提示音：上行三音琶音，表示系统就绪 */
    Buzzer_Init();
    Buzzer_StartupSound();
 
    // Main loop for the Debug task
    while (1)
    {
        WS2812_CtrlHSV(hue, WS2812_SAT, WS2812_VAL);
        hue = (uint16_t)((hue + WS2812_HUE_STEP) % 360U);
        osDelay(WS2812_FRAME_MS); // Delay to prevent busy waiting

    }
}