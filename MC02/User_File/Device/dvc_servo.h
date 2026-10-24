#ifndef DVC_SERVO_H
#define DVC_SERVO_H


#include "main.h"
#include <stdint.h>


typedef struct {
    float start_angle;    	    //从什么角度开始
    uint32_t range;         	//舵机的旋转范围
    void (*SetServoAngle)(float angle);  // 设置当前角度的函数指针
} Servo_t;

extern Servo_t claw;
extern Servo_t gimbal;
extern Servo_t tray;

void Set_Claw_Angle(float angle);
void Set_Gimbal_Angle(float angle);
void Set_Tray_Angle(float angle);

void Servo_Init(void);
void Servo_SetAngle_270(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle);
void Servo_SetAngle_180(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle);
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us);

#endif // DVC_SERVO_H