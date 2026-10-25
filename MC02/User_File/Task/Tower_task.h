#ifndef TOWER_TASK_H
#define TOWER_TASK_H

#include "main.h"

void Tower_Init(void);
void Tower_Task(void *argument);

void Tower_SetLift_Pos(int16_t target_pos,uint16_t speed,uint8_t acc);
void Tower_SetTelescope_Pos(int16_t target_pos,uint16_t speed,uint8_t acc);

#endif // TOWER_TASK_H  