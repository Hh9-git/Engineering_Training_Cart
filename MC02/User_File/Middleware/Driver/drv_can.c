#include "drv_can.h"
#include <stdlib.h>

//----------------------BSP_FDCAN----------------------//
#ifdef HAL_FDCAN_MODULE_ENABLED

 /*使用局部变量，避免静态全局变量互相覆盖*/
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
void FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Buf)
{
   
    if ((Buf != NULL))
    {
        FDCAN_TxHeader.Identifier = ID;                         // 目标ID
        FDCAN_TxHeader.IdType = FDCAN_STANDARD_ID;              // 标准ID
        FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;          // 数据帧格式
        FDCAN_TxHeader.DataLength = FDCAN_DLC_BYTES_8;          // 数据长度
        FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  //
        FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
        FDCAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 传统CAN模式
        FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
        FDCAN_TxHeader.MessageMarker = 0;                       //

        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_TxHeader, Buf) ;
    }
}
#endif /* HAL_FDCAN_MODULE_ENABLED */


//--------------------------------------------------------------------------------------------------------------------
// FDCAN Interrupt
#ifdef HAL_FDCAN_MODULE_ENABLED

static FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
static uint8_t FDCAN_RxBuf[8];

static uint8_t FDCAN_Function_Count = 0;        // 函数数量
static FDCAN_Interrupt_t *FDCAN_ItSource_Array; // CAN中断回调函数结构体数组指针

/**
 * FDCAN过滤器配置
 * 如果使用了AttachInterrupt_FDCAN函数则无需再使用
 */
inline void FDCAN_FilterInit(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef FDCAN_FilterInitStructure;
    /*配置FIFO0的全通滤波器*/
    FDCAN_FilterInitStructure.IdType = FDCAN_STANDARD_ID;             // 标准ID
    FDCAN_FilterInitStructure.FilterIndex = 0;                        // 过滤器索引
    FDCAN_FilterInitStructure.FilterType = FDCAN_FILTER_MASK;         // 过滤器类型
    FDCAN_FilterInitStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
    FDCAN_FilterInitStructure.FilterID1 = 0x000;                      // 11位ID
    FDCAN_FilterInitStructure.FilterID2 = 0x000;                      // 11位掩码
    HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterInitStructure);
    /*使能FDCAN的硬件中断*/
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能新消息接收中断，必须开，否则收不到报文
    /*调试阶段全开检查总线质量，稳定后可以根据需要关掉多余的*/
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_EVT_FIFO_ELT_LOST, 0);//发送事件FIFO溢出丢失，诊断用，不影响收发
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RAM_ACCESS_FAILURE, 0);  //FDCAN内部消息RAM访问失败，诊断用
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_DATA_PROTOCOL_ERROR, 0);//CAN总线数据段的位填充错误、CRC错误、位错误，诊断用
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_ARB_PROTOCOL_ERROR, 0);//多个节点同时发送引起的位错误（仲裁失败）、格式错，诊断用
    
}

// 联接CAN中断源和中断回调函数，
void AttachInterrupt_FDCAN(FDCAN_HandleTypeDef *hfdcan, void (*FDCAN_Callback)(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf))
{
    /*动态回调注册机制*/
    FDCAN_ItSource_Array = (FDCAN_Interrupt_t *)realloc(FDCAN_ItSource_Array, (FDCAN_Function_Count + 1) * sizeof(FDCAN_Interrupt_t));  // 重新分配内存空间
    FDCAN_ItSource_Array[FDCAN_Function_Count].hfdcan = hfdcan;                                                                         //记录FDCAN句柄
    FDCAN_ItSource_Array[FDCAN_Function_Count].FDCAN_Callback = FDCAN_Callback;                                                         //记录回调函数指针
    FDCAN_Function_Count++;                                                                                                             //数组大小计数+1

    /*FDCAN接收过滤器初始化*/
    FDCAN_FilterInit(hfdcan); 
    /*启动FDCAN*/
    HAL_FDCAN_Start(hfdcan);
}

// FDCAN中断回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        assert_param(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxBuf) == HAL_OK);

        for (uint8_t i = 0; i < FDCAN_Function_Count; i++)
        {
            if (FDCAN_ItSource_Array[i].hfdcan == hfdcan)
            {
                FDCAN_ItSource_Array[i].FDCAN_Callback(&FDCAN_RxHeader, FDCAN_RxBuf);
            }
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        assert_param(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN_RxHeader, FDCAN_RxBuf) == HAL_OK);

        for (uint8_t i = 0; i < FDCAN_Function_Count; i++)
        {
            if (FDCAN_ItSource_Array[i].hfdcan == hfdcan)
            {
                FDCAN_ItSource_Array[i].FDCAN_Callback(&FDCAN_RxHeader, FDCAN_RxBuf);
            }
        }
    }
}

#endif /* HAL_FDCAN_MODULE_ENABLED */