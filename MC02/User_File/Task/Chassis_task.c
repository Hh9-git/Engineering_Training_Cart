#include "Chassis_task.h"
#include "cmsis_os2.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include <math.h>
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

uint8_t acc_vel=230;
uint8_t acc_pos=0;
int16_t  car_move_speed = 50;

#define CHASSIS_WHEEL_DIAMETER_MM   80.0f      // 麦克纳姆轮直径
#define CHASSIS_PULSE_PER_REV       3200.0f    // Emm_V5：16 细分 × 200 步 = 3200 脉冲/圈
// 每毫米行程对应的脉冲数：3200 / (π × 80) ≈ 12.73
#define CHASSIS_PULSE_PER_MM  (CHASSIS_PULSE_PER_REV / (3.14159265f * CHASSIS_WHEEL_DIAMETER_MM))

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

    Chassis_Move_PosY(1000.0f);    // 单次定距：1000mm

    for (;;)
    {
        osDelay(10);               // 速度模式调试时改回 Chassis_Move(...)
    }
}


void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W)
{
    int16_t speed[5];

    speed[1] = +Vy +Vx -W;
	speed[2] = -Vy + Vx +W;
	speed[3] = -Vy + Vx -W;
	speed[4] = +Vy + Vx +W;

    Chassis_SetMotor_Vel(speed);
}


// x: 行程(mm)，正数沿 +x 横移
void Chassis_Move_PosX(float x)
{
    int16_t speed[5];

    if (x == 0.0f) { return; }                              // 空指令，同时避免除零

    int16_t  v_x = (x > 0.0f) ? car_move_speed : -car_move_speed;
    uint32_t clk = (uint32_t)(fabsf(x) * CHASSIS_PULSE_PER_MM + 0.5f);  // 脉冲数

	speed[1] = +v_x ;
	speed[2] = +v_x ;
	speed[3] = +v_x ;
	speed[4] = +v_x ;

    Chassis_SetMotor_Pos(speed, clk);

}

void Chassis_Move_PosY(float y)
{
    int16_t speed[5];

    if (y == 0.0f) { return; }                              // 空指令，同时避免除零

    int16_t  v_y = (y > 0.0f) ? car_move_speed : -car_move_speed;
    uint32_t clk = (uint32_t)(fabsf(y) * CHASSIS_PULSE_PER_MM + 0.5f);  // 脉冲数

    speed[1] = +v_y ;
    speed[2] = -v_y ;
    speed[3] = -v_y ;
    speed[4] = +v_y ;

    Chassis_SetMotor_Pos(speed, clk);

}








void Chassis_SetMotor_Pos(int16_t speed[5],uint32_t clk)
{
    //正向
    if(speed[1]>=0){Emm_V5_Pos_Control(4, 0, speed[1], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(4, 1, -speed[1], acc_pos, clk, 0, true);}
	osDelay(1);

    //正向
	if(speed[2]>=0){Emm_V5_Pos_Control(3, 0, speed[2], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(3, 1, -speed[2], acc_pos, clk, 0, true);}
	osDelay(1);

    //正向
	if(speed[3]>=0){Emm_V5_Pos_Control(1, 1, speed[3], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(1, 0, -speed[3], acc_pos, clk, 0, true);}
	osDelay(1);

    //正向
	if(speed[4]>=0){Emm_V5_Pos_Control(2, 1, speed[4], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(2, 0, -speed[4], acc_pos, clk, 0, true);}
    osDelay(1);

    Emm_V5_Synchronous_motion(0); //广播触发
}

void Chassis_SetMotor_Vel(int16_t speed[5])//此处用5，方便与电机对应1，2，3，4
{
    //正向
    if(speed[1]>=0){Emm_V5_Vel_Control(4, 0, speed[1], acc_vel, false);}else{Emm_V5_Vel_Control(4, 1, -speed[1], acc_vel, false);}
	osDelay(1);

    //正向
	if(speed[2]>=0){Emm_V5_Vel_Control(3, 0, speed[2], acc_vel, false);}else{Emm_V5_Vel_Control(3, 1, -speed[2], acc_vel, false);}
	osDelay(1);

    //正向
	if(speed[3]>=0){Emm_V5_Vel_Control(1, 1, speed[3], acc_vel, false);}else{Emm_V5_Vel_Control(1, 0, -speed[3], acc_vel, false);}
	osDelay(1);

    //正向
	if(speed[4]>=0){Emm_V5_Vel_Control(2, 1, speed[4], acc_vel, false);}else{Emm_V5_Vel_Control(2, 0, -speed[4], acc_vel, false);}
    osDelay(1);
}