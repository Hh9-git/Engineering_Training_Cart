#include "Main_task.h"

#include "dvc_vofa.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"

uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};

void Main_Init(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

void Main_Task(void *argument)
{
	Main_Init();

	for (;;)
	{
	
		osDelay(10);
	}
}
