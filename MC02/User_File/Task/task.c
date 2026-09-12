#include "task.h"

uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};


void Task_Init(void)
{
   bsp_can_init();

}

void Task_Loop(void)
{
    	fdcanx_send_data(&hfdcan1, 0x520, tx_data, 8);
		HAL_Delay(100);

	    fdcanx_send_data(&hfdcan2, 0x521, tx_data, 8);
		HAL_Delay(100);

		// fdcanx_send_data(&hfdcan3, 0x520, tx_data, 8);
		// HAL_Delay(100);
       
        //    vofa_start();
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
}