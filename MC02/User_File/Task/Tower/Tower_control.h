
#ifndef TOWER_CONTROL_H
#define TOWER_CONTROL_H

#include "main.h"



#define LIFT_PULSE_PER_MM 83.33f
#define TELESCOPE_PULSE_PER_MM 25.0f


typedef struct
{
    int16_t target_pos;
    uint16_t speed;
    uint8_t acc;
    uint8_t update_flag;
    uint32_t ack_base;      /* 发命令前的位置应答计数，用来确认电机真收到了 */
    uint8_t retry;          /* 已经重发了几次，防止电机不在线时无限刷总线 */
} Tower_Motor_Target_t;


void Tower_Control_Init(void);
void Tower_Motor_Run(void);
void Tower_Control(uint32_t yaw,
                   uint32_t z,
                   uint16_t z_speed,
                   uint32_t y,
                   uint16_t y_speed);
void Tower_SetGimbal_Angle(int16_t angle);
void Tower_SetClaw_Angle(int16_t angle);

void Tower_SetLift_Pos(int16_t target_pos,
                       uint16_t speed,
                       uint8_t acc);

void Tower_SetTelescope_Pos(int16_t target_pos,
                            uint16_t speed,
                            uint8_t acc);


#endif
