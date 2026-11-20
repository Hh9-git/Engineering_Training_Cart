
#ifndef TOWER_CONTROL_H
#define TOWER_CONTROL_H

#include "main.h"

void Tower_Control_Init(void);

void Tower_SetGimbal_Angle(int16_t angle);

void Tower_SetLift_Pos(int16_t target_pos,
                       uint16_t speed,
                       uint8_t acc);

void Tower_SetTelescope_Pos(int16_t target_pos,
                            uint16_t speed,
                            uint8_t acc);

void Tower_Motor_Homing(void);
#endif
