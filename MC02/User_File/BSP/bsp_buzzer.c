#include "bsp_buzzer.h"

#define BUZZER_TIM          htim12
#define BUZZER_CHANNEL      TIM_CHANNEL_2

/* 计数器时钟固定成 1MHz，之后只用 ARR 调频率：ARR = 1MHz / freq。
 * 16 位 ARR 因此可覆盖约 15Hz ~ 500kHz，整个音频段绰绰有余；
 * 换音调时只动 ARR、不动 PSC，计算简单也不会溢出。 */
#define BUZZER_COUNTER_HZ   1000000UL

void Buzzer_Init(void)
{
    /* 先把占空比清 0 再开 PWM，否则上电瞬间会用 CubeMX 里的初始值响一下 */
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, 0U);

    HAL_TIM_Base_Start(&BUZZER_TIM);
    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_SetTone(uint16_t freq_hz)
{
    if (freq_hz == 0U)
    {
        Buzzer_Off();
        return;
    }

    /* TIM12 挂 APB1，APB1 预分频不为 1 时定时器时钟 = 2 × PCLK1。
     * 用运行时查询而不是写死常数，这样以后在 CubeMX 里改时钟树不会失效
     * （算法与 Core/Src/stm32h7xx_hal_timebase_tim.c 里的一致）。 */
    uint32_t prescaler = ((2U * HAL_RCC_GetPCLK1Freq()) / BUZZER_COUNTER_HZ) - 1U;
    uint32_t period    = BUZZER_COUNTER_HZ / freq_hz;

    if (period < 2U)     period = 2U;        /* 保证 ARR >= 1 */
    if (period > 65536U) period = 65536U;    /* TIM12 的 ARR 是 16 位 */

    __HAL_TIM_SET_PRESCALER(&BUZZER_TIM, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, period - 1U);
    /* 50% 占空比 —— 无源蜂鸣器在这个占空比下声压最大 */
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_CHANNEL, period / 2U);

    /* 立刻把新的 PSC/ARR 装进影子寄存器：PSC 原本要等当前计数周期结束才生效，
     * 连续换音时会听到拖尾。UG 事件同时重载 PSC 和 ARR。
     * HAL_TIM_PWM_Start() 没开更新中断，所以这里不会触发中断。 */
    BUZZER_TIM.Instance->EGR = TIM_EGR_UG;

    /* "设完就响"：即使之前被 Buzzer_Off() 停过通道，这里也重新打开。
     * HAL_TIM_PWM_Start() 对已启动的通道是幂等的。 */
    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_On(void)
{
    /* 不改变音调，按当前 CCR 恢复发声 */
    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_Off(void)
{
    HAL_TIM_PWM_Stop(&BUZZER_TIM, BUZZER_CHANNEL);
}

void Buzzer_StartupSound(void)
{
    /* C5-E5-G5-C6 上行琶音，最后落在高八度的 C6 上收尾。
     * 想换旋律只改这两张表即可 —— 音高和时长一一对应。 */
    static const uint16_t note_hz[] = { 523U, 659U, 784U, 1047U };   /* C5, E5, G5, C6 */
    static const uint16_t note_ms[] = {  95U,  95U,  95U,  300U };

    for (uint32_t i = 0; i < (sizeof(note_hz) / sizeof(note_hz[0])); i++)
    {
        Buzzer_SetTone(note_hz[i]);
        osDelay(note_ms[i]);

        /* 音符之间留一点间隙，否则连在一起会听不出是三个音 */
        Buzzer_Off();
        osDelay(35U);
    }
}