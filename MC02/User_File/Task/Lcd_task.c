#include "Lcd_task.h"
#include "dvc_lcd.h"


void Lcd_Init(void)
{
    LCD_Init(); // 初始化 LCD
    LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, BLACK); // 清屏
}

void Lcd_Task(void *argument)
{
    (void)argument;

    Lcd_Init();
    for (;;)
    {
        LCD_ShowString(120, 72,(uint8_t *)"hello", BLUE, BLACK, 24, 0);
        // 在这里添加 LCD 任务的功能
        osDelay(100); // 延迟以防止忙等待
    }
}