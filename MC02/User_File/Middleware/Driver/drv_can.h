#ifndef DRV_CAN_H
#define DRV_CAN_H


#include "main.h"

//---------------------------------------------------------------------------------------------------------------------
// 如果启用FDCAN功能
#ifdef HAL_FDCAN_MODULE_ENABLED
#include "fdcan.h"

    typedef struct
    {
        FDCAN_HandleTypeDef *hfdcan;
        void (*FDCAN_Callback)(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf);

    } FDCAN_Interrupt_t;

    extern void FDCAN_FilterInit(FDCAN_HandleTypeDef *hfdcan);
    extern void FDCAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Buf);
    extern void AttachInterrupt_FDCAN(FDCAN_HandleTypeDef *hfdcan, void (*FDCAN_Callback)(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf));
#endif /* HAL_FDCAN_MODULE_ENABLED */

//-----------------------------------------------------------------------------------------------------------------------



#endif // DRV_CAN_H
