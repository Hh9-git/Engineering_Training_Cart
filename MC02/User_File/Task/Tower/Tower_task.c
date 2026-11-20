
#include "Tower_task.h"
#include "Tower_control.h"
#include "cmsis_os2.h"
#include "dvc_stepmotor.h"

void Tower_Init(void)
{
    Tower_Control_Init();
}

void Tower_Task(void *argument)
{
    (void)argument;

    Tower_Init();

    Tower_SetGimbal_Angle(0);

    // 示例：夹紧物料
    // Tower_SetClaw_Angle(45);

    for (;;)
    {
        osDelay(100);
    }
}
