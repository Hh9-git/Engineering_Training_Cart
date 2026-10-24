#include "Lcd_task.h"
#include "IMU_task.h"
#include "Chassis_task.h"
#include "dvc_lcd.h"
#include <math.h>

#define RAD_TO_DEG  57.29578f   // 弧度转角度

#define SIZE_DATA   16          // 数据区字号，字符宽度为 SIZE_DATA/2
#define X_LEFT      8           // 左栏(原始数据)起始 x
#define X_RIGHT     148         // 右栏(姿态角/锁头)起始 x
#define X_VAL_L     (X_LEFT + 16)   // 左栏数值列，标签 1 字符
#define X_VAL_LW    (X_LEFT + 40)   // 左栏数值列，标签 4 字符
#define X_VAL_R     (X_RIGHT + 48)  // 右栏数值列，标签最多 5 字符
#define X_VAL_RL    (X_RIGHT + 16)  // 右栏数值列，标签 1 字符

// 陀螺零偏量级只有 0.001~0.02 rad/s，必须用 3 位小数才看得见
#define DEC_GYRO    3
#define DEC_DEG     1
#define DEC_ACC     2

// 右栏各行的 y 坐标，20px 一行
#define Y_YAW       40
#define Y_PITCH     60
#define Y_ROLL      80
#define Y_TARGET    100
#define Y_ERR       120
#define Y_VZ        140
#define Y_LOCK      160
#define Y_TEMP      184
#define Y_STATUS    208

// 静态标题只画一次，循环里只刷新数值
static void Lcd_DrawLabels(void)
{
    LCD_ShowString(X_RIGHT, Y_YAW,    (const uint8_t *)"Yaw",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_PITCH,  (const uint8_t *)"Pit",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_ROLL,   (const uint8_t *)"Rol",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_TARGET, (const uint8_t *)"Tgt",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_ERR,    (const uint8_t *)"Err",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_VZ,     (const uint8_t *)"Vz",    WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_LOCK,   (const uint8_t *)"Lock",  WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_RIGHT, Y_TEMP,   (const uint8_t *)"Temp",  WHITE, BLACK, SIZE_DATA, 0);
}

void Lcd_Init(void)
{
    LCD_Init(); // 初始化 LCD
    LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, BLACK); // 清屏
    Lcd_DrawLabels();
}

void Lcd_Task(void *argument)
{
    Lcd_Init();
    for (;;)
    {
       
        osDelay(100); // 延迟以防止忙等待
    }
}