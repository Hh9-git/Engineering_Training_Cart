#include "Tower_task.h"
#include "dvc_stepmotor.h"

void Tower_Init(void)
{
    // //1上行 0下降

    //  Emm_V5_Angle_Control(5, 1,1000, 0, 360, 0, false);
    //  osDelay(100); 


    //  //夹爪伸出0缩回1
    // Emm_V5_Angle_Control(6,0, 1000, 0, 45, 0, false);
    //  osDelay(100); 



   
}


void Tower_Task(void *argument)
{
    Tower_Init();
    // Main loop for the Tower task
    while (1)
    {
      
        // Add the functionality for the Tower task here
        osDelay(100); // Delay to prevent busy waiting
    }
}
    