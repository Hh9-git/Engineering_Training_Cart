#include "Chassis_task.h"
#include "dvc_stepmotor.h"


void Chassis_Init(void)
{

    AttachInterrupt_FDCAN(&hfdcan1, ZDT_CAN_Callback);
    // Emm_V5_En_Control(2, true, true);                 // 使能电机
    osDelay(500);
}

void Chassis_Task(void *argument)
{
    (void)argument;
    Chassis_Init();

  
    for (;;)
    {
        
        osDelay(10);
    }
}

