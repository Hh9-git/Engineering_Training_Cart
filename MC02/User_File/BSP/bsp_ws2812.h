#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include "main.h"

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

/* hue: 0~359，sat/val: 0~255。内部转成 RGB 后调用 WS2812_Ctrl。
 * 注意：SPI6 句柄是单 owner 的，只允许同一个任务调用，
 *       不要从多个任务或中断里并发调用。 */
void WS2812_CtrlHSV(uint16_t hue, uint8_t sat, uint8_t val);

/* 彩色渐变（"呼吸灯"）的一帧：不自带延时，画完立即返回。
 * 由调用者的任务循环负责节拍，例如：
 *     for (;;) { WS2812_BreathStep(); osDelay(WS2812_BREATH_FRAME_MS); }
 * 色相每帧推进 2 度，10ms 一帧 -> 1.8s 走完一圈。 */
#define WS2812_BREATH_FRAME_MS  10U
void WS2812_BreathStep(void);

/* 诊断工具：闪 n 次白光，用来在没有示波器/调试器时把灯珠当成状态输出。
 * 内部用 DWT 周期计数器忙等，不依赖任何中断 —— 所以 HAL 时基被屏蔽时
 * 它依然能正常闪烁（这正是定位 FreeRTOS 启动阶段问题时的关键）。
 * 平时不用；确实需要看启动到哪一级时，在各级插入 WS2812_Beacon(n) 即可。 */
void WS2812_Beacon(uint32_t n);

#endif /* BSP_WS2812_H */