#ifndef DVC_REMOTE_H
#define DVC_REMOTE_H

#include "main.h"
#include "drv_usart.h"

#define SBUS_UART huart5

/****FS-i6X遥控器数据结构体****/
typedef struct{       //低位~中值~高位
    float Right_X;  //286~1024~1746
    float Right_Y;  //240~1024~1678
    float Left_Y;   //303~1024~1765
    float Left_X;   //248~1024~1700
    float SWA;      //240~1807
    float SWB;      //240~1807
    float SWC;      //240~1024~1807
    float SWD;      //240~1807
    float VRA;      //298~1024~1805
    float VRB;      //240~1807
}Remote_control_struct;

extern uint8_t Rx_buf[64];
extern uint16_t sbus_channels[16];
extern Remote_control_struct Remote_control_FS;
//函数定义

void SBUS_Init(void (*SBUS_Function)(void));

void SBUS_callback(uint8_t *data, uint16_t size);
#endif //DVC_REMOTE_H