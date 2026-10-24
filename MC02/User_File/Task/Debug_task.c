#include "Debug_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "bsp_buzzer.h"

void Debug_Init(void)
{


}

void Debug_Task(void *argument)
{
    Debug_Init();

    for (;;)
    {
        WS2812_BreathStep();                  // 画一帧彩色渐变
        osDelay(WS2812_BREATH_FRAME_MS);      // 10ms 一帧 -> 1.8s 走完一圈
    }
}