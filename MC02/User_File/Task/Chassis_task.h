#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"

void Chassis_Init(void);
void Chassis_Task(void *argument);

void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W);

void Chassis_Move_PosX(float x);
void Chassis_Move_PosY(float y);

void Chassis_SetMotor_Vel(int16_t speed[5]);  

void Chassis_SetMotor_Pos(int16_t speed[5], uint32_t clk);

#endif // CHASSIS_TASK_H