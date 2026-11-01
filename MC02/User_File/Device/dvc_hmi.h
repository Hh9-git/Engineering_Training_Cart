#ifndef DVC_HMI_H
#define DVC_HMI_H

#include "main.h"


#define TJC_UART huart7
#define TJC_UART_INS UART7
extern UART_HandleTypeDef huart7;



void tjc_send_string(char* str);
void tjc_send_txt(char* objname, char* attribute, char* txt);
void tjc_send_val(char* objname, char* attribute, int val);
void tjc_send_nstring(char* str, unsigned char str_length);
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);

void HMI_Init(void);
void HMI_RxCpltISR(void);
void HMI_Callback(uint8_t *pData, uint16_t size);
HAL_StatusTypeDef HMI_Send_String(char*name,char*data);
HAL_StatusTypeDef HMI_Send_Number(char*name,int num);
HAL_StatusTypeDef HMI_Send_Float(char*name,float  num);

#define RINGBUFFER_LEN	(512)     


#define usize getRingBufferLength()
#define code_c() initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x) read1ByteFromRingBuffer(x)


extern uint8_t RxBuffer[1];
extern uint32_t msTicks;


#endif /* DVC_HMI_H */