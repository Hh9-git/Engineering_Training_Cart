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

     

    // /* 初始姿态 */
    // Tower_Target_Update(0, 100, 5);
    // Tower_Claw_on();


    for (;;)
    {
        /* 底层电机和舵机控制 */
        Tower_Motor_Run();

        /* 上层动作状态机 */
        Tower_Action_Run();

        osDelay(10);
    }
}