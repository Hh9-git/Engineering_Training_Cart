#include "drv_pwm.h"

void PWM_Init(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void PWM_SetDuty(void)
{
    /*TIM1和TIM2的Period=19999，50%占空比=10000*/
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 10000);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 10000);

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 10000);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 10000);
}