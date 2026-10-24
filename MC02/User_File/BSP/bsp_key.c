#include "bsp_key.h"
#include "bsp_buzzer.h"

KeyState keyState = NO_DETECT;
uint32_t overTick = 0;

int8_t BSP_UserKey_Detect(void)
{
    GPIO_PinState pinSta;
    uint32_t nowTick = HAL_GetTick();
    pinSta = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    if (keyState == NO_DETECT)
    {
        if (pinSta == GPIO_PIN_RESET)
        {
            keyState = DETECTING;
          
            overTick = nowTick + 100;//0.1s
            
        }
        return BUTTON_NOT_PRESSED;
    }
    else if (keyState == DETECTING)
    {
        if (nowTick >= overTick)
        {
            if (pinSta == GPIO_PIN_RESET)
            {
                keyState = DETECTED;
                return BUTTON_PRESSED;
            }
            else
            {
                keyState = NO_DETECT;
                return BUTTON_NOT_PRESSED;
            }
        }
        return BUTTON_NOT_PRESSED;
    }
    else if (keyState == DETECTED)
    {
        if (pinSta == GPIO_PIN_SET)
        {
            keyState = NO_DETECT;
        }
        return BUTTON_NOT_PRESSED;
    }
    return BUTTON_NOT_PRESSED;
}



void Key_Task(void *argument)
{

 /* Infinite loop */
    for(;;)
    {
        if (BSP_UserKey_Detect() == BUTTON_PRESSED)
        {
           Buzzer_StartupSound();
        }
        // vbus = (adc_val[0]*3.3f/65535)*11.0f;
        osDelay(10);
    }

}