#include "dvc_remote.h"

#define SBUS_START 0x0F
#define SBUS_END 0x00
#define Rx_buf_size 64
/******遥控器数据缓冲区********/
uint8_t Rx_buf[Rx_buf_size] = {0};

/********SBUS配置-100000波特率，9位数据位，偶校验，1位停止位*********/
/*********以上配置仅为FS-i6X配置，其他可能需要修改***********/
uint16_t sbus_channels[16];

/******定义FS-i6X遥控器数据结构体变量******/
Remote_control_struct Remote_control_FS;
void (*RemoteFunction)(void);

void SBUS_callback(uint8_t *data, uint16_t size)
{
	if (Rx_buf[0] == SBUS_START&& Rx_buf[24] == SBUS_END)
	{

		/********遥控器操作接码**********/
		Remote_control_FS.Right_X= (uint16_t)((Rx_buf[1] | Rx_buf[2] << 8) & 0x07FF);
		Remote_control_FS.Right_Y= (uint16_t)((Rx_buf[2] >> 3 | Rx_buf[3] << 5) & 0x07FF);
		Remote_control_FS.Left_Y= (uint16_t)((Rx_buf[3] >> 6 | Rx_buf[4] << 2 | Rx_buf[5] << 10) & 0x07FF);
		Remote_control_FS.Left_X = (uint16_t)((Rx_buf[5] >> 1 | Rx_buf[6] << 7) & 0x07FF);
		Remote_control_FS.VRA = (uint16_t)((Rx_buf[6] >> 4 | Rx_buf[7] << 4) & 0x07FF);
		Remote_control_FS.VRB= (uint16_t)((Rx_buf[7] >> 7 | Rx_buf[8] << 1 | Rx_buf[9] << 9) & 0x07FF);
		Remote_control_FS.SWA= (uint16_t)((Rx_buf[9] >> 2 | Rx_buf[10] << 6) & 0x07FF);
		Remote_control_FS.SWB= (uint16_t)((Rx_buf[10] >> 5 | Rx_buf[11] << 3) & 0x07FF);
		Remote_control_FS.SWC= (uint16_t)((Rx_buf[12] | Rx_buf[13] << 8) & 0x07FF);
		Remote_control_FS.SWD = (uint16_t)((Rx_buf[13] >> 3 | Rx_buf[14] << 5) & 0x07FF);
		/* 调试打印已移到 Debug_Task 走 USB：UART_Print 固定发到 PRINT_UART = huart7，
		   而那正是串口屏的串口，且这里在中断里以 HAL_MAX_DELAY 阻塞发送，会打断屏指令。 */

		/*********sbus协议解码************/
		// sbus_channels[0] = (uint16_t)((Rx_buf[1] | Rx_buf[2] << 8) & 0x07FF);
		// sbus_channels[1] = (uint16_t)((Rx_buf[2] >> 3 | Rx_buf[3] << 5) & 0x07FF);
		// sbus_channels[2] = (uint16_t)((Rx_buf[3] >> 6 | Rx_buf[4] << 2 | Rx_buf[5] << 10) & 0x07FF);
		// sbus_channels[3] = (uint16_t)((Rx_buf[5] >> 1 | Rx_buf[6] << 7) & 0x07FF);
		// sbus_channels[4] = (uint16_t)((Rx_buf[6] >> 4 | Rx_buf[7] << 4) & 0x07FF);
		// sbus_channels[5] = (uint16_t)((Rx_buf[7] >> 7 | Rx_buf[8] << 1 | Rx_buf[9] << 9) & 0x07FF);
		// sbus_channels[6] = (uint16_t)((Rx_buf[9] >> 2 | Rx_buf[10] << 6) & 0x07FF);
		// sbus_channels[7] = (uint16_t)((Rx_buf[10] >> 5 | Rx_buf[11] << 3) & 0x07FF);
		// sbus_channels[8] = (uint16_t)((Rx_buf[12] | Rx_buf[13] << 8) & 0x07FF);
		// sbus_channels[9] = (uint16_t)((Rx_buf[13] >> 3 | Rx_buf[14] << 5) & 0x07FF);
		// sbus_channels[10] = (uint16_t)((Rx_buf[14] >> 6 | Rx_buf[15] << 2 | Rx_buf[16] << 10) & 0x07FF);
		// sbus_channels[11] = (uint16_t)((Rx_buf[16] >> 1 | Rx_buf[17] << 7) & 0x07FF);
		// sbus_channels[12] = (uint16_t)((Rx_buf[17] >> 4 | Rx_buf[18] << 4) & 0x07FF);
		// sbus_channels[13] = (uint16_t)((Rx_buf[18] >> 7 | Rx_buf[19] << 1 | Rx_buf[20] << 9) & 0x07FF);
		// sbus_channels[14] = (uint16_t)((Rx_buf[20] >> 2 | Rx_buf[21] << 6) & 0x07FF);
		// sbus_channels[15] = (uint16_t)((Rx_buf[21] >> 5 | Rx_buf[22] << 3) & 0x07FF);
		// UART_Print("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",sbus_channels[0],sbus_channels[1],sbus_channels[2],sbus_channels[3],sbus_channels[4],sbus_channels[5],sbus_channels[6],sbus_channels[7],sbus_channels[8],sbus_channels[9]);
	}
	else // 调整错误帧
	{
		for (uint8_t i = 1; i < Rx_buf_size; i++)
		{
			if (Rx_buf[i] == SBUS_START && Rx_buf[i - 1] == SBUS_END)
			{
				HAL_UARTEx_ReceiveToIdle_DMA(&SBUS_UART,Rx_buf,Rx_buf_size);
				break;
			}
		}
	}
	RemoteFunction();
}

void SBUS_Init(void (*SBUS_Function)(void))
{
	RemoteFunction = SBUS_Function;
	/*********遥控测试**********/
	AttachInterrupt_UART_DMA(&SBUS_UART,Rx_buf,64,SBUS_callback);
}