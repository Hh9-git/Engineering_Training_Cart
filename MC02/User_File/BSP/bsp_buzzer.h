#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/* 设置音调并立即发声。freq_hz 传 0 等同于 Buzzer_Off()。
 * 无源蜂鸣器：音调由 PWM 频率决定，可听范围大约 20Hz ~ 20kHz。 */
void Buzzer_SetTone(uint16_t freq_hz);

/* 启动提示音：上行四音琶音 C5-E5-G5-C6，听感是"就绪"。
 * 阻塞约 0.72s（内部用 osDelay），必须在任务里调用；
 * 调用前要先 Buzzer_Init()。 */
void Buzzer_StartupSound(void);

#endif /* BSP_BUZZER_H */