#ifndef DVC_SERVO_H
#define DVC_SERVO_H


#include "tim.h"

void Servo_Init(void);
void Servo_SetAngle_90(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle);
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us);

#endif // DVC_SERVO_H