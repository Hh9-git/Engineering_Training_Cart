#include "drv_usart.h"
#include "dvc_yis320.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

//----------------------BSP_UART----------------------//
#ifdef HAL_UART_MODULE_ENABLED
char UART_TxBuf[128]={0};
// 串口打印 UART_Print("123456\r\n");格式需要加\r
int UART_Print(char *fmt, ...)
{
    int ret;
    va_list ap;
    va_start(ap, fmt);
    ret = vsprintf(UART_TxBuf, fmt, ap);
    va_end(ap);
    if (ret > 0)
    {
        HAL_UART_Transmit(&PRINT_UART, (uint8_t *)UART_TxBuf, ret, HAL_MAX_DELAY);
    }
    return ret;
}
#endif /* HAL_UART_MODULE_ENABLED */

//--------------------------------------------------------------------------------------------------------------------
// UART Interrupt
#ifdef HAL_UART_MODULE_ENABLED

static uint8_t UART_DMA_Function_Count = 0;           // DMA函数数量
static UART_DMA_Interrupt_t *UART_DMA_ItSource_Array; // UART中断回调函数结构体数组指针

/**
 * 联接UART DMA和中断回调函数
 * 应输入UART源, 回调函数个数以及回调函数的地址
 * 联接后自动开启DMA
 */
void AttachInterrupt_UART_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t RxBuf_Size,void (*UART_Callback)(uint8_t *pData, uint16_t size))
{
    UART_DMA_ItSource_Array = (UART_DMA_Interrupt_t *)realloc(UART_DMA_ItSource_Array, (UART_DMA_Function_Count + 1) * sizeof(UART_DMA_Interrupt_t));
    UART_DMA_ItSource_Array[UART_DMA_Function_Count].huart = huart;
    UART_DMA_ItSource_Array[UART_DMA_Function_Count].UART_RxBuf = pData;
    UART_DMA_ItSource_Array[UART_DMA_Function_Count].UART_RxBuf_Size = RxBuf_Size;
    UART_DMA_ItSource_Array[UART_DMA_Function_Count].UART_Callback = UART_Callback;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, UART_DMA_ItSource_Array[UART_DMA_Function_Count].UART_RxBuf, RxBuf_Size);
    UART_DMA_Function_Count++;
}

// UART空闲中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < UART_DMA_Function_Count; i++)
        {
            if (UART_DMA_ItSource_Array[i].huart == huart)
            {
                UART_DMA_ItSource_Array[i].UART_Callback(UART_DMA_ItSource_Array[i].UART_RxBuf, Size);
                HAL_UARTEx_ReceiveToIdle_DMA(huart, UART_DMA_ItSource_Array[i].UART_RxBuf, UART_DMA_ItSource_Array[i].UART_RxBuf_Size);
            }
        }
}

/* 帧错误/噪声/溢出都会进这里。HAL 的处理是"停掉接收 DMA 然后回调"，
 * 而弱定义的 HAL_UART_ErrorCallback 是空的 —— 也就是说线上抖一下，
 * 这块串口从此再也收不到数据（表现为"发了没反应"，而且不报错）。
 * 这里只记下是哪个串口，真正的重挂交给任务做：
 * 中断里此刻 DMA abort 可能还没走完，直接重启会拿到 HAL_BUSY。 */
static UART_HandleTypeDef *UART_RxError_Source = NULL;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* YIS320 走单字节中断：ORE 后 HAL 会停掉接收，
     * 而且 ORE 与 RXNE 同时置位时错误分支会把刚重挂好的接收再停一次，
     * 所以必须在这里补挂，不能只靠 RxCplt。 */
    if (huart->Instance == USART10)
    {
        IMU_OnUartError(huart->ErrorCode);
        return;
    }

    UART_RxError_Source = huart;
}

void UART_DMA_Recover_Error(void)
{
    if (UART_RxError_Source == NULL)
    {
        return;
    }

    UART_HandleTypeDef *huart = UART_RxError_Source;
    UART_RxError_Source = NULL;

    for (uint8_t i = 0; i < UART_DMA_Function_Count; i++)
    {
        if (UART_DMA_ItSource_Array[i].huart == huart)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                         UART_DMA_ItSource_Array[i].UART_RxBuf,
                                         UART_DMA_ItSource_Array[i].UART_RxBuf_Size);
            break;
        }
    }
}

#endif /* HAL_UART_MODULE_ENABLED */