#include "Main_task.h"
#include "Chassis_task.h"
#include "dvc_vofa.h"
#include "dvc_stepmotor.h"
#include "bsp_buzzer.h"
#include "bsp_ws2812.h"


void Main_Init(void)
{
	/* 启动提示音：上行三音琶音，表示系统就绪 */
    Buzzer_Init();
    Buzzer_StartupSound();
}

/* 等待当前直线/旋转动作结束（带超时，避免异常时卡死） */
static void Main_WaitMotion(void)
{
	uint32_t t0 = osKernelGetTickCount();

	while ((chassis.line.active || chassis.rotate_active) &&
	       ((osKernelGetTickCount() - t0) < 10000u))
	{
		osDelay(10);
	}
}

void Main_Task(void *argument)
{
	Main_Init();


	Chassis_Absolute_Pos_Ctrl_Line(-200, 200, 10);
	Main_WaitMotion();
	/* 测试序列只执行一次，每步等动作完成再发下一条 */
	Chassis_Absolute_Pos_Ctrl_Line(-200, 1075, 20);
	Main_WaitMotion();

	Chassis_Rotate(90, 255);
	Main_WaitMotion();

	Chassis_Absolute_Pos_Ctrl_Line(-1100, 1075, 20);
	Main_WaitMotion();

	Chassis_Rotate(180, 255);
	Main_WaitMotion();


	Chassis_Absolute_Pos_Ctrl_Line(-1075, 100, 20);
	Main_WaitMotion();

	Chassis_Rotate(90, 255);
	Main_WaitMotion();


	Chassis_Absolute_Pos_Ctrl_Line(-100, 100, 10);
	Main_WaitMotion();



	Chassis_Rotate(0, 255);
	Main_WaitMotion();
	
	//回启动区
	Chassis_Absolute_Pos_Ctrl_Line(180, -85, 20);
	Main_WaitMotion();


	Chassis_Rotate(0, 255);
	Main_WaitMotion();


	for (;;)
	{
		osDelay(100);       /* 序列跑完就停在这里 */
	}
}
