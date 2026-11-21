#include "Lcd_task.h"
#include "IMU_task.h"
#include "Chassis_task.h"
#include "dvc_lcd.h"
#include "dvc_yis320.h"
#include "bsp_ws2812.h"
#include <math.h>
#include <stdio.h>

#define LCD_REFRESH_MS  200     // LCD 走 SPI 逐字画点，一次刷 20 多个字符约 8ms，200ms 刷一次够看

#define SIZE_DATA   16          // 数据区字号，字符宽度为 SIZE_DATA/2
/* 屏幕右半部分物理碎裂，所有内容统一排在左半屏(x < 140) */
#define X_LABEL     8           // 标题列起始 x
#define X_VALUE     56          // 数值列起始 x（标题最多 5 字符 = 40px，再留 8px 间距）

// 陀螺零偏量级只有 0.001~0.02 rad/s，必须用 3 位小数才看得见
#define DEC_GYRO    3
#define DEC_DEG     1
#define DEC_ACC     2

// 各行的 y 坐标，20px 一行
#define Y_YAW       40
#define Y_PITCH     60
#define Y_ROLL      80
#define Y_TARGET    100
#define Y_ERR       120
#define Y_VZ        140
#define Y_LOCK      160
#define Y_TEMP      184
#define Y_YIS       208     // YIS320 yaw（该行在加 YIS 之前一直没被使用）
#define Y_PKT       222     // YIS320 通过帧数 / 校验失败数

/* YIS320 两行用 12 号字：左半屏只有 140px，16 号字一行放不下 8 个字符 */
#define SIZE_SMALL  12

// 静态标题只画一次，循环里只刷新数值
static void Lcd_DrawLabels(void)
{
    LCD_ShowString(X_LABEL, Y_YAW,    (const uint8_t *)"Yaw",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_PITCH,  (const uint8_t *)"Pit",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_ROLL,   (const uint8_t *)"Rol",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_TARGET, (const uint8_t *)"Tgt",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_ERR,    (const uint8_t *)"Err",   WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_VZ,     (const uint8_t *)"Vz",    WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_LOCK,   (const uint8_t *)"Lock",  WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_TEMP,   (const uint8_t *)"Temp",  WHITE, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_LABEL, Y_YIS,    (const uint8_t *)"YIS",   WHITE, BLACK, SIZE_SMALL, 0);
    LCD_ShowString(X_LABEL, Y_PKT,    (const uint8_t *)"PKT",   WHITE, BLACK, SIZE_SMALL, 0);
}

/* 没有数据源的行先用占位符标出来，免得看成是"卡在 0" */
static void Lcd_DrawPlaceholders(void)
{
    LCD_ShowString(X_VALUE, Y_TARGET, (const uint8_t *)"--", GRAY, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_VALUE, Y_ERR,    (const uint8_t *)"--", GRAY, BLACK, SIZE_DATA, 0);
    LCD_ShowString(X_VALUE, Y_VZ,     (const uint8_t *)"--", GRAY, BLACK, SIZE_DATA, 0);
}

/* 浮点转成以 0.1 为单位的定点整数，并按四舍五入取整 */
static int32_t ToDeci(float v)
{
    return (int32_t)(v * 10.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
}

/* 显示 x.x 格式的定点数；尾部补空格，盖掉上一次更长时留下的字符。
   不用 LCD_ShowFloatNum：它会按固定位数补前导 0，yaw 会显示成 005.0。 */
static void Lcd_ShowDeci(uint16_t x, uint16_t y, int32_t val10, uint8_t sizey)
{
    char buf[16];
    int32_t mag = (val10 < 0) ? -val10 : val10;

    sprintf(buf, "%s%ld.%ld   ", (val10 < 0) ? "-" : "",
            (long)(mag / 10), (long)(mag % 10));
    LCD_ShowString(x, y, (const uint8_t *)buf, WHITE, BLACK, sizey, 0);
}

static void Lcd_UpdateValues(void)
{
    char buf[24];

    Lcd_ShowDeci(X_VALUE, Y_YAW,   ToDeci(imuAngle[INS_YAW_ADDRESS_OFFSET]   * RAD_TO_DEG), SIZE_DATA);
    Lcd_ShowDeci(X_VALUE, Y_PITCH, ToDeci(imuAngle[INS_PITCH_ADDRESS_OFFSET] * RAD_TO_DEG), SIZE_DATA);
    Lcd_ShowDeci(X_VALUE, Y_ROLL,  ToDeci(imuAngle[INS_ROLL_ADDRESS_OFFSET]  * RAD_TO_DEG), SIZE_DATA);
    Lcd_ShowDeci(X_VALUE, Y_TEMP,  ToDeci(imuTemp), SIZE_DATA);

    /* YIS320：yaw 在首帧后自动归零，单位已经是度 */
    Lcd_ShowDeci(X_VALUE, Y_YIS, ToDeci(IMU_getYaw()), SIZE_SMALL);

    /* 通过帧数 / 校验失败数，用来判断接收链路是否正常 */
    sprintf(buf, "%lu/%lu   ", (unsigned long)imu_pkt_cnt, (unsigned long)imu_cs_err);
    LCD_ShowString(X_VALUE, Y_PKT, (const uint8_t *)buf, WHITE, BLACK, SIZE_SMALL, 0);
}

void Lcd_Init(void)
{
    LCD_Init(); // 初始化 LCD
    LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, BLACK); // 清屏
    Lcd_DrawLabels();
    Lcd_DrawPlaceholders();
}

void Lcd_Task(void *argument)
{
    Lcd_Init();

    uint32_t last     = HAL_GetTick();
    uint8_t  last_cal = 0xFF;       // 0xFF：保证第一轮一定画一次标定状态

    for (;;)
    {
        if (HAL_GetTick() - last >= LCD_REFRESH_MS)
        {
            last = HAL_GetTick();

            Lcd_UpdateValues();

            /* 标定状态只在变化时重画，省 SPI 带宽 */
            if (imu_calibrated != last_cal)
            {
                last_cal = imu_calibrated;
                if (imu_calibrated)
                {
                    LCD_ShowString(X_VALUE, Y_LOCK, (const uint8_t *)"OK  ", GREEN,  BLACK, SIZE_DATA, 0);
                }
                else
                {
                    LCD_ShowString(X_VALUE, Y_LOCK, (const uint8_t *)"CALI", YELLOW, BLACK, SIZE_DATA, 0);
                }
            }
        }
        osDelay(20);
    }
}