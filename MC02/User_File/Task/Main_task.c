#include "Main_task.h"
#include "Chassis_task.h"
#include "Tower/Tower_action.h"
#include "dvc_stepmotor.h"
#include "bsp_buzzer.h"
#include "bsp_ws2812.h"
#include "Tower_task.h"
#include "Tower_control.h"
#include "Tower_action.h"


void Main_Init(void)
{
	/* 启动提示音：上行三音琶音，表示系统就绪 */
    // Buzzer_Init();
    // Buzzer_StartupSound();
}

void Main_Task(void *argument)
{
	Main_Init();


	/* 每个航点：先直线走到坐标，再原地转到目标航向；
	   time_ds=0 表示按距离自动算速度，非 0 表示指定运动时长(0.1s) */
	// Chassis_Go_Pos(-200,200,   0, 0, 0);

	// /*第一趟*/
	// //出启动区
	// Chassis_Go_Pos(-150,150,   0, 0, 0);
	// //扫码
	// Chassis_Go_Pos(-150,  1065,  0, 5, 1);
	// // osDelay(50);
	// //拐角
	// Chassis_Go_Pos(-150, 2000, 90, 5, 0);
	// //原料区
	// Chassis_Go_Pos(-1065,  2000,  90, 5, 1);
	// // osDelay(100);

    // //前往粗加工区掉头
	// Chassis_Go_Pos(-1100,  2000,  180, 0, 0);
	// //到达粗加工区加变向，塔吊面对粗加工区
	// Chassis_Go_Pos(-1100,  50,  180, 15, 1);
	// Chassis_Go_Pos(-1040,  50,  -90, 0, 1);

	// //前往暂存区
	// Chassis_Go_Pos(-2000,50, 180,5, 0);
	// Chassis_Go_Pos(-2000,1065, 180, 5,1);




	// /*第二趟*/
    // //前往原料区
	// Chassis_Go_Pos(-2000, 2000, 90, 5, 0);
	// //原料区
	// Chassis_Go_Pos(-1065,  2000,  90, 5, 1);
    // //前往粗加工区掉头
	// Chassis_Go_Pos(-1100,  2000,  180, 0, 0);
	// //到达粗加工区加变向，塔吊面对粗加工区
	// Chassis_Go_Pos(-1100,  50,  180, 15, 1);
	// Chassis_Go_Pos(-1040,  50,  -90, 0, 1);

	// //前往暂存区
	// Chassis_Go_Pos(-2000,50, 180,5, 0);
	// Chassis_Go_Pos(-2000,1065, 180, 5,1);

	// //前往原料区变向
	// Chassis_Go_Pos(-2000, 2000, 90, 5, 0);
	// //停止区
	// Chassis_Go_Pos(-100,  2000,  0, 5, 1);




	// Chassis_Go_Pos(0, 0, 0,10,1);
	// Chassis_Go_Pos(-1075,  200,  90, 0, 0);

	// Chassis_Go_Pos(-200,   200,   0, 0, 0);
	// Chassis_Go_Pos(0,    0,   0, 0, 0);   /* 回启动区 */

    Tower_Action_Start(TOWER_ACTION_RESET);  // 启动时先复位塔吊姿态



	// osDelay(1000);  // 等待机构移动到位
    // Chassis_Lock();



	


	// Tower_Grab_Tray(2);  // 测试抓取中间位置
	// Tower_Put_Ground(2);  // 测试放置中间位置

	// Tower_Grab_Tray(1);  // 测试抓取中间位置
	// Tower_Put_Ground(1);  // 测试放置中间位置

	// Tower_Grab_Tray(3);  // 测试抓取中间位置
	// Tower_Put_Ground(3);  // 测试放置中间位置



	// Tower_Grab_Ground(1);  // 测试抓取中间位置
	// Tower_Put_Car(1);  // 测试放置中间位置

	// Tower_Grab_Ground(2);  // 测试抓取左侧位置
	// Tower_Put_Car(2);  // 测试放置左侧位置

	// Tower_Grab_Ground(3);  // 测试抓取右侧位置
	// Tower_Put_Car(3);  // 测试放置右侧位置
	for (;;)
	{
		
		osDelay(100);
	}
}
