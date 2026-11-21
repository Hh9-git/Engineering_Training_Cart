
#include "Tower_task.h"
#include "Tower_control.h"
#include "Tower_action.h"
#include "dvc_stepmotor.h"

void Tower_Init(void)
{
    Tower_Control_Init();
}

void Tower_Task(void *argument)
{
    (void)argument;

    Tower_Init();
    Tower_Claw_off();  // 夹爪闭合

    for (;;)
    {
        Tower_Motor_Run();
        osDelay(100);
    }
}



