#include "dvc_servo.h"
#include "drv_pwm.h"


Servo_t claw={0,180,Set_Claw_Angle};
Servo_t gimbal={-110,270,Set_Gimbal_Angle};
Servo_t tray={135,270,Set_Tray_Angle};



void Servo_Init(void)
{
         PWM_Init();

         Set_Claw_Angle(claw.start_angle);
         Set_Gimbal_Angle(gimbal.start_angle);
         Set_Tray_Angle(tray.start_angle);

         //开启舵机5v电源
         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}



void Set_Claw_Angle(float angle)
{
    Servo_SetAngle_180(&htim2, TIM_CHANNEL_1, angle);
}

void Set_Gimbal_Angle(float angle)
{
    Servo_SetAngle_270(&htim2, TIM_CHANNEL_3, angle);
}

void Set_Tray_Angle(float angle)
{
    Servo_SetAngle_270(&htim1, TIM_CHANNEL_1, angle);
}


















void Servo_SetAngle_270(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle)
{
    if (angle > 135)  angle = 135;
    if (angle < -135) angle = -135;

    // 270° 舵机：-135°=500μs, 0°=1500μs, +135°=2500μs, 1计数=1μs
    // 2000μs / 270° ≈ 7.407μs/度
    uint32_t ccr = 1500 + (angle * 1000) / 135;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


void Servo_SetAngle_180(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle)
{
    if (angle > 90)  angle = 90;
    if (angle < -90) angle = -90;

    // 180° 舵机：-90°=500μs, 0°=1500μs, +90°=2500μs, 1计数=1μs
    // 2000μs / 180° ≈ 11.11μs/度
    uint32_t ccr = 1500 + (angle * 1000) / 90;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us)
{
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    __HAL_TIM_SET_COMPARE(htim, channel, us);
}