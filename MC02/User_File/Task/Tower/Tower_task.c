#include "Tower_task.h"
#include "Tower_control.h"
#include "Tower_action.h"
#include "dvc_stepmotor.h"

void Tower_Init(void)
{
    Tower_Control_Init();
    Tower_Action_Init();
}
void Tower_Task(void *argument)
{
    (void)argument;

    Tower_Init();
    
    for (;;)
    {
        /* 底层电机和舵机控制 */
        Tower_Motor_Run();

        /* 上层动作状态机 */
        Tower_Action_Run();

        osDelay(10);
    }
}