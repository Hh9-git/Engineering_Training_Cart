#include "Tower_task.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"
#include "dvc_lcd.h"
#include "bsp_ws2812.h"


void Tower_Init(void)
{
    //  Emm_V5_Angle_Control(5, 1,1000, 0, 360, 0, false);
    //  osDelay(100); 


    //  
    // Emm_V5_Angle_Control(6,0, 1000, 0, 45, 0, false);
    //  osDelay(100); 
    // Tower_SetLift_Pos(50, 200, 0);
    // osDelay(10);
    // Tower_SetTelescope_Pos(50, 200, 0);
    Servo_Init();
}


void Tower_Task(void *argument)
{
    Tower_Init();
  
    while (1)
    {
      
        Set_Claw_Angle(-90); // Set claw angle to 90 degrees
       
        osDelay(100); // Delay to prevent busy waiting
    }
}
    

//增量式
void Tower_SetLift_Pos(int16_t target_pos,uint16_t speed,uint8_t acc)
{


    if(target_pos >=135){
		target_pos =135;
	}
	// if (target_pos <0){
    //     LCD_ShowString(1, 1, (const uint8_t *)"error:OUT", WHITE, BLACK, 16, 0);
    //     while(1){}
    // }

     // 计算需要的脉冲数（取绝对值）
    uint32_t pulses = (uint32_t)(fabsf(target_pos) * 80);


    //1上行 0下降
    uint8_t dir = (target_pos > 0) ? 1 : 0;


    Emm_V5_Pos_Control(5, dir, speed, acc, pulses, 0, false);
    osDelay(10); 
}

void Tower_SetTelescope_Pos(int16_t target_pos,uint16_t speed,uint8_t acc)
{


    if(target_pos >=135){
        target_pos =135;}


// 计算需要的脉冲数（取绝对值）
    uint32_t pulses = (uint32_t)(fabs(target_pos) * 25.465f);

    uint8_t dir = (target_pos > 0) ? 0 : 1;    //夹爪伸出0缩回1
    
   Emm_V5_Pos_Control(6, dir, speed, acc, pulses, 0, false);
    osDelay(10); 
}