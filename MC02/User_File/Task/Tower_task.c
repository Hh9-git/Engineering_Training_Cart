#include "Tower_task.h"

void Tower_Init(void)
{
    // Initialization code for the Tower task can be added here
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
    