#include "task.h"
#include "drv_can.h"
#include "fdcan.h"
#include <sys/_types.h>

uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};


void Task_Init(void)
{
//    bsp_can_init();

	//FDCAN_FilterInit(&hfdcan1);
   AttachInterrupt_FDCAN(&hfdcan1,NULL);
}

void Task_Loop(void)
{
	    // UART_Print("123456\r\n");
	    // vofa_demo();

		// HAL_UART_Transmit(&huart1, tx_data, 8, 1000);



		FDCAN_Transmit(&hfdcan1, 0x200, tx_data);

		HAL_Delay(100);
		FDCAN_Transmit(&hfdcan1, 0x234, tx_data);


    	// fdcanx_send_data(&hfdcan1, 0x520, tx_data, 8);
		// HAL_Delay(100);

		// fdcanx_send_data(&hfdcan1, 0x346, tx_data, 8);
		// HAL_Delay(100);

	    // fdcanx_send_data(&hfdcan2, 0x521, tx_data, 8);
		// HAL_Delay(100);

		// // fdcanx_send_data(&hfdcan3, 0x520, tx_data, 8);
		// // HAL_Delay(100);
       
        // //    vofa_start();
        // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
}
