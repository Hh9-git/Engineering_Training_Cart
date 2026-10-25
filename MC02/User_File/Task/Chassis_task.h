#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

void Chassis_Init(void);
void Chassis_Task(void *argument);

void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W);

void Chassis_SetMotor(int16_t speed[5]);  // 设置底盘电机速度，speed[1]~speed[4] 分别对应 1~4 号电机

#endif // CHASSIS_TASK_H