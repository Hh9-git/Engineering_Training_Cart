#include "Main_task.h"
#include "dvc_vofa.h"
#include "dvc_stepmotor.h"
#include "bsp_buzzer.h"


void Main_Init(void)
{
	/* 启动提示音：上行三音琶音，表示系统就绪 */
    Buzzer_Init();
    Buzzer_StartupSound();
}

void Main_Task(void *argument)
{
	Main_Init();

	for (;;)
	{
	
		osDelay(10);
	}
}
