#include "bsp_ws2812.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF8     // 1码

#define WS2812_RESET_BYTES 250      // 复位脉冲字节数：250B / 6MHz = 333us 低电平
#define WS2812_SPI_TIMEOUT 0x20     // 短超时(ms)

static uint8_t       ws2812_txbuf[24];
static const uint8_t ws2812_reset[WS2812_RESET_BYTES];   /* 全 0，latch/复位脉冲 */

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 8; i++)
    {
        ws2812_txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        ws2812_txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        ws2812_txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }

    if (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY)
    {
        HAL_SPI_Abort(&WS2812_SPI_UNIT);
        WS2812_SPI_UNIT.State = HAL_SPI_STATE_READY; 
    }

    HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_txbuf, 24, WS2812_SPI_TIMEOUT);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_reset, WS2812_RESET_BYTES, WS2812_SPI_TIMEOUT);
}

static void WS2812_HSV2RGB(uint16_t hue, uint8_t sat, uint8_t val,
                           uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (sat == 0U)
    {
        *r = *g = *b = val;             /* 灰度 */
        return;
    }

    hue %= 360U;
    uint8_t sector = (uint8_t)(hue / 60U);                          /* 0~5 */
    uint8_t frac   = (uint8_t)(((hue % 60U) * 255U) / 60U);         /* 0~255 */

    uint8_t p = (uint8_t)(((uint16_t)val * (255U - sat)) / 255U);
    uint8_t q = (uint8_t)(((uint16_t)val * (255U - ((uint16_t)sat * frac) / 255U)) / 255U);
    uint8_t t = (uint8_t)(((uint16_t)val * (255U - ((uint16_t)sat * (255U - frac)) / 255U)) / 255U);

    switch (sector)
    {
        case 0: *r = val; *g = t;   *b = p;   break;   /* 红 -> 黄 */
        case 1: *r = q;   *g = val; *b = p;   break;   /* 黄 -> 绿 */
        case 2: *r = p;   *g = val; *b = t;   break;   /* 绿 -> 青 */
        case 3: *r = p;   *g = q;   *b = val; break;   /* 青 -> 蓝 */
        case 4: *r = t;   *g = p;   *b = val; break;   /* 蓝 -> 品红 */
        default:*r = val; *g = p;   *b = q;   break;   /* 品红 -> 红 */
    }
}

void WS2812_CtrlHSV(uint16_t hue, uint8_t sat, uint8_t val)
{
    uint8_t r, g, b;

    WS2812_HSV2RGB(hue, sat, val, &r, &g, &b);
    WS2812_Ctrl(r, g, b);
}


#define WS2812_HUE_STEP     2U
#define WS2812_SAT          255U
#define WS2812_VAL          150U


void WS2812_BreathStep(void)
{
    static uint16_t hue = 0U;

    WS2812_CtrlHSV(hue, WS2812_SAT, WS2812_VAL);
    hue = (uint16_t)((hue + WS2812_HUE_STEP) % 360U);
}
