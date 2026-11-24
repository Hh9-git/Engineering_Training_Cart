#ifndef DVC_SERVO_H
#define DVC_SERVO_H


#include "main.h"


typedef struct {
    float start_angle;    	    //从什么角度开始
    uint32_t range;         	//舵机的旋转范围
    void (*SetServoAngle)(int16_t angle);  // 设置当前角度的函数指针
} Servo_t;

/* 舵机运动插值结构体 */
typedef struct
{
    float start_angle;          // 起始角度
    float current_angle;        // 当前角度
    float target_angle;         // 目标角度

    uint32_t start_tick;        // 运动开始时间
    uint32_t last_update_tick;  // 上次更新时间

    uint16_t interval;          // 更新周期(ms)
    uint16_t move_time;         // 运动总时间(ms)

    uint8_t moving_flag;        // 运动状态

} Servo_Motion_t;


extern Servo_t claw;
extern Servo_t gimbal;
extern Servo_t tray;


// 爪子舵机：180°
void Set_Claw_Angle(int16_t angle);
// 云台舵机：270°
void Set_Gimbal_Angle(int16_t angle);
void Servo_Gimbal_Update(void);
// 转盘舵机：270°
void Set_Tray_Angle(int16_t angle);


void Servo_Init(void);
void Servo_SetAngle_270(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle);
void Servo_SetAngle_180(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle);
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us);

#endif // DVC_SERVO_H