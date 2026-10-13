#include "bsp_ws2812.h"

/* SPI6 = 6MHz，1 个 SPI 位 = 166.7ns，每位灯珠数据用 8 个 SPI 位编码。
 * 写入时 >>1 只是把上升沿推迟 1 位，不改变高电平宽度。
 *   0码 0xC0>>1 = 0x60 = 0110 0000 -> 高 2 位 = 333ns  (WS2812B 规格 250~550ns)
 *   1码 0xF8>>1 = 0x7C = 0111 1100 -> 高 5 位 = 833ns  (WS2812B 规格 650~950ns)
 * 原 1 码用 0xF0(高 4 位=667ns)，只比 650ns 下限高 2.6%，实测灯珠把 1 码也判成了 0 码
 * （整条帧变全 0，灯珠熄灭）。放宽到 5 位后裕量约 28%。
 * 位周期仍是 8 位 = 1.333us，两码的低电平 833ns / 333ns 也都在规格内。 */
#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF8     // 1码

#define WS2812_RESET_BYTES 250      // 复位脉冲字节数：250B / 6MHz = 333us 低电平
#define WS2812_SPI_TIMEOUT 0x20     // 短超时(ms)，绝不在这里无限等

/* SPI6 句柄是单 owner 的，只允许同一个任务调用 */
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

    /* 原代码这里是 `HAL_SPI_Transmit(..., Size=0, ...)` 加一句无超时的
     * `while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);`：
     *   - Size=0 在 HAL 里直接 return HAL_ERROR，是个纯空操作；
     *   - 那句 while 才是致命的：任何一次发送超时，HAL 都会把 State 置成
     *     HAL_SPI_STATE_ERROR（不是 READY），于是这句 while 永久锁死 CPU，
     *     灯珠停在最后一个成功刷新的颜色上，且此后所有发送静默失效。
     * 改成：先恢复状态，再带超时发送。 */
    if (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY)
    {
        HAL_SPI_Abort(&WS2812_SPI_UNIT);
        WS2812_SPI_UNIT.State = HAL_SPI_STATE_READY;   /* 兜底，绝不让上层卡住 */
    }

    HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_txbuf, 24, WS2812_SPI_TIMEOUT);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, ws2812_reset, WS2812_RESET_BYTES, WS2812_SPI_TIMEOUT);
}

/* 整数版 HSV -> RGB，无浮点，不占额外栈。
 * sector 把色环分成 6 段，每段 60 度；frac 是段内插值(0~255)。
 * p/q/t 是标准写法：p 是饱和度对应的最小值，q/t 是段内的两个过渡值。 */
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

/* ---------------- 诊断用信标 ---------------- */

/* 用 DWT 周期计数器做忙等延时，完全不依赖任何中断。
 * 这是刻意为之：当前调试阶段 HAL 时基(TIM23)可能被 BASEPRI 屏蔽，
 * 一旦用它做延时，诊断工具自己就会被卡死，什么也看不到。 */
static void WS2812_BusyDelayMs(uint32_t ms)
{
    static uint8_t s_dwt_ready = 0;

    if (!s_dwt_ready)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能 DWT 跟踪 */
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动周期计数 */
        s_dwt_ready = 1U;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = ms * (SystemCoreClock / 1000U);

    while ((DWT->CYCCNT - start) < ticks) { }
}

/**
  * @brief  用灯珠闪 n 次白光，报告系统跑到第几级。
  * @note   全程不依赖中断，所以在 HAL 时基被屏蔽时依然能正常显示。
  */
void WS2812_Beacon(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        WS2812_Ctrl(255, 255, 255);   /* 白 */
        WS2812_BusyDelayMs(120U);
        WS2812_Ctrl(0, 0, 0);         /* 灭 */
        WS2812_BusyDelayMs(220U);
    }
}