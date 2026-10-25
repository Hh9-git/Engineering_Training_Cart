#include "Chassis_task.h"
#include "cmsis_os2.h"
#include "dvc_stepmotor.h"
#include <stdint.h>


/** 
        底盘移动坐标系    W方向遵循坐标系右手法则
        ^y
        |                   
        |______>x          


        底盘俯视图
        
速度解算俯视图
 Speed[1]          Speed[2]                  
    \               /
      \           /
        \       /
          \   /
            *
          /    \
        /        \
      /            \
    /                \
Speed[3]           Speed[4]


        
实际底盘电机ID俯视图
 0x04               0x03                |
    \               /                   |
      \           /                     |
        \       /   {  }                   
          \   /    {tower}{claw}
            *       {  }
          /    \
        /        \
      /            \
    /     {tray}     \
0x01                0x02

*/

uint8_t a=230;

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
        Chassis_Move(0, 0, 10); // 停止运动
        osDelay(10);
    }
}





void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W)
{
    int16_t speed[5];

    speed[1] = +Vy +Vx -W;
	speed[2] = -Vy + Vx +W;
	speed[3] = -Vy + Vx -W;
	speed[4] = +Vy + Vx +W;

    Chassis_SetMotor(speed);


}


void Chassis_SetMotor(int16_t speed[5])
{
    //正向
    if(speed[1]>=0){Emm_V5_Vel_Control(4, 0, speed[1], a, false);}else{Emm_V5_Vel_Control(4, 1, -speed[1], a, false);}
	osDelay(1);

    //正向
	if(speed[2]>=0){Emm_V5_Vel_Control(3, 0, speed[2], a, false);}
	else{
	    Emm_V5_Vel_Control(3, 1, -speed[2], a, false);  
	}
	osDelay(1);
	if(speed[3]>=0){
		Emm_V5_Vel_Control(1, 1, speed[3], a, false);//正向	
	}
	else{
	    Emm_V5_Vel_Control(1, 0, -speed[3], a, false);  
	}
	osDelay(1);
	if(speed[4]>=0){
		Emm_V5_Vel_Control(2, 1, speed[4], a, false);//正向	
	}
	else{
	    Emm_V5_Vel_Control(2, 0, -speed[4], a, false); 
	}
    osDelay(1);
}