#include "dvc_servo.h"

void Servo_Init(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    /******所有通道恢复中位（1500μs）*********/
    Servo_SetAngle(&htim1, TIM_CHANNEL_1, 0);
    Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
    Servo_SetAngle(&htim2, TIM_CHANNEL_1, 0);
    Servo_SetAngle(&htim2, TIM_CHANNEL_3, 0);
}


/**
 * @brief  270° 舵机角度控制，适配 TIM1/TIM2 (PSC=239, ARR=19999)
 * @param  htim   : TIM句柄 (&htim1 / &htim2)
 * @param  channel: TIM_CHANNEL_1/3
 * @param  angle  : -135 ~ +135 度（0 为中位）
 */
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, int16_t angle)
{
    if (angle > 135)  angle = 135;
    if (angle < -135) angle = -135;

    // 270° 舵机：-135°=500μs, 0°=1500μs, +135°=2500μs, 1计数=1μs
    // 2000μs / 270° ≈ 7.407μs/度
    uint32_t ccr = 1500 + (angle * 1000) / 135;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


/**
 * @brief  直接设置高电平时间（单位：微秒 us），适配 PSC=239, ARR=19999
 * @param  htim   : TIM句柄 (&htim1 / &htim2)
 * @param  channel: TIM_CHANNEL_1/3
 * @param  us     : 500 ~ 2500 us
 */
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t us)
{
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    __HAL_TIM_SET_COMPARE(htim, channel, us);
}