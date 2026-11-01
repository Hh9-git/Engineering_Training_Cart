#include "main.h"
#include "usart.h"
#include "dvc_hmi.h"
#include "dvc_yis320.h"

/* 全工程唯一的 UART 接收完成回调：按 Instance 分发。
 * 各驱动只提供自己的 ISR 处理函数，互不引用；
 * 以后再加串口只需要在这里加一个分支。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == TJC_UART_INS)        /* UART7：串口屏 */
    {
        HMI_RxCpltISR();
    }
    else if (huart->Instance == USART10)        /* USART10：YIS320 */
    {
        IMU_RxCpltISR();
    }
}