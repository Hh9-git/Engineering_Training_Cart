#include "task.h"

uint8_t tx_data[8] = {0,1,2,3,4,5,6,7};


void Task_Init(void)
{
	// PWM_Init();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	Servo_Init();
	// Servo_SetAngle_90(&htim1, TIM_CHANNEL_1, 0);
   	AttachInterrupt_FDCAN(&hfdcan1,NULL);

	FDCAN_FilterInit_Ext(&hfdcan1);                    // 覆盖为扩展帧全通
    HAL_Delay(1000);

    // Emm_V5_En_Control(2, true, true);                 // 使能电机
    // HAL_Delay(100);
    // Emm_V5_Trig_Encoder_Cal(2);
   	// HAL_Delay(100);


	// Emm_V5_Vel_Control(2, 0, 1000, 100, false);  // 速度模式，顺时针，速度1000RPM，加速度100，非同步
	// HAL_Delay(1000);
//    Emm_V5_Trig_Encoder_Cal(3);

	// Emm_V5_Pos_Control(3, 0, 1000, 100, 1000, 0, false);
    // Emm_V5_Pos_Control(2, 0, 1000, 100, 10000, 0, false); // 位置模式，顺时针，速度1000RPM，加速度100，脉冲数10000，相对上一输入目标位置进行相对位置运动，非同步
	
	
	Emm_V5_Angle_Control(2, 0, 1000, 100, 90, 0, false); // 角度模式，顺时针，速度1000RPM，加速度100，角度90°，相对上一输入目标位置进行相对位置运动，非同步
	HAL_Delay(100);


//     Emm_V5_Stop_Now(2, false);
}

void Task_Loop(void)
{
	// Servo_SetAngle_90(&htim1, TIM_CHANNEL_1, 45);        // PE9 转 45°
	// Servo_SetAngle_90(&htim1, TIM_CHANNEL_3, -90);       // PE13 转 -90°
	// Servo_SetAngle_90(&htim2, TIM_CHANNEL_1, 0);         // PA0 回中
	// Servo_SetPulse(&htim2, TIM_CHANNEL_3, 2000); 
	//	Servo_SetAngle_90(&htim1, TIM_CHANNEL_1, 60);
}
