#include "visual_identity.h"
#include "drv_usart.h"

#define VISUAL_RX_BUF_SIZE  16

static uint8_t visual_rx_buf[VISUAL_RX_BUF_SIZE];


static void Visual_UART_RxCallback(uint8_t *pData, uint16_t size)
{
    // 接收到视觉模块数据后的处理
}