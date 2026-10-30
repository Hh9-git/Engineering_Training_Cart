#include "dvc_hmi.h"
#include "drv_usart.h"

#include <stdio.h>
#include <stdarg.h>
#include <sys/_intsup.h>




typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
}RingBuffer_t;

RingBuffer_t ringBuffer;	//创建一个ringBuffer的缓冲区
uint8_t RxBuffer[1];


#define FRAME_LENGTH 7

void HMI_Init(void)
{
    HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);
    initRingBuffer();
}

/* 发一条完整指令到串口屏：ASCII 正文 + 0xFF 0xFF 0xFF */
static HAL_StatusTypeDef HMI_Send_Format(const char *fmt, ...)
{
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 3, fmt, ap);
    va_end(ap);

    if (len <= 0) return HAL_ERROR;
    if (len > (int)sizeof(buf) - 4) len = sizeof(buf) - 4;

    buf[len++] = (char)0xFF;
    buf[len++] = (char)0xFF;
    buf[len++] = (char)0xFF;

    return HAL_UART_Transmit(&TJC_UART, (uint8_t *)buf, (uint16_t)len, 10);
}

/**
     * 串口屏的指令必须是"对象名.属性=值"，少了 .val 屏幕会静默丢弃整条指令，

    示例：
    n0.val=10        HMI_Send_Number("n0.val", 10) ;
    t0.txt="hello"   HMI_Send_String("t0.txt","hello");
    
*/

HAL_StatusTypeDef HMI_Send_String(char*name,char*data)
{
    return HMI_Send_Format("%s=\"%s\"", name, data);
}

HAL_StatusTypeDef HMI_Send_Number(char*name,int num)
{
    return HMI_Send_Format("%s=%d", name, num);
}


HAL_StatusTypeDef HMI_Send_Float(char*name,float  num)
{
    return HMI_Send_Format("%s=%d", name, (int)(num*100));
}


/********************************************************
函数名：  		intToStr
日期：    	2024.09.18
功能：    	将整形转换为字符串
输入参数：		要转换的整形数据,输出的字符串数组
返回值： 		无
修改记录：
**********************************************************/
void intToStr(int num, char* str) {
    int i = 0;
    int isNegative = 0;

    // 处理负数
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 提取每一位数字
    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    // 如果是负数，添加负号
    if (isNegative) {
        str[i++] = '-';
    }

    // 添加字符串终止符
    str[i] = '\0';

    // 反转字符串
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    return ;
}



/********************************************************
函数名：  		uart_send_char
日期：    	2024.09.18
功能：    	串口发送单个字符
输入参数：		要发送的单个字符
返回值： 		无
修改记录：
**********************************************************/
void uart_send_char(char ch)
{
	uint8_t ch2 = (uint8_t)ch;
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
	//while(__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TXE) == RESET);	//等待发送完毕
	while(__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TC) == RESET);
    //发送单个字符
	HAL_UART_Transmit_IT(&TJC_UART, &ch2, 1);
	return;
}


void uart_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	return;
}

/********************************************************
函数名：  		tjc_send_string
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
示例:			tjc_send_val("n0", "val", 100); 发出的数据就是 n0.val=100
修改记录：
**********************************************************/
void tjc_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
函数名：  		tjc_send_txt
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
示例:			tjc_send_txt("t0", "txt", "ABC"); 发出的数据就是t0.txt="ABC"
修改记录：
**********************************************************/
void tjc_send_txt(char* objname, char* attribute, char* txt)
{

    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_string("=\"");
    uart_send_string(txt);
    uart_send_char('\"');
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}


/********************************************************
函数名：  		tjc_send_val
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
修改记录：
**********************************************************/
void tjc_send_val(char* objname, char* attribute, int val)
{
	//拼接字符串,比如n0.val=123
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_char('=');
    //C语言中整形的取值范围是：“-2147483648 ~ 2147483647”, 最长为-2147483648,加上结束符\0一共12个字符
    char txt[12]="";
    intToStr(val, txt);
    uart_send_string(txt);
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
函数名：  		tjc_send_nstring
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串,字符串长度
返回值： 		无
修改记录：
**********************************************************/
void tjc_send_nstring(char* str, unsigned char str_length)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    for (int var = 0; var < str_length; ++var)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}





/********************************************************
函数名：  	HAL_UART_RxCpltCallback
日期：    	2022.10.08
功能：    	串口接收中断,将接收到的数据写入环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == TJC_UART_INS)	// 判断是由哪个串口触发的中断
	{
		write1ByteToRingBuffer(RxBuffer[0]);
		HAL_UART_Receive_IT(&TJC_UART,RxBuffer,1);		// 重新使能串口2接收中断
	}
	return;
}



/********************************************************
函数名：  		initRingBuffer
日期：    	2022.10.08
功能：    	初始化环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void initRingBuffer(void)
{
	//初始化相关信息
	ringBuffer.Head = 0;
	ringBuffer.Tail = 0;
	ringBuffer.Length = 0;
	return;
}



/********************************************************
函数名：  		write1ByteToRingBuffer
日期：    	2022.10.08
功能：    	往环形缓冲区写入数据
输入参数：		要写入的1字节数据
返回值： 		void
修改记录：
**********************************************************/
void write1ByteToRingBuffer(uint8_t data)
{
	if(ringBuffer.Length >= RINGBUFFER_LEN) //判断缓冲区是否已满
	{
	return ;
	}
	ringBuffer.Ring_data[ringBuffer.Tail]=data;
	ringBuffer.Tail = (ringBuffer.Tail+1)%RINGBUFFER_LEN;//防止越界非法访问
	ringBuffer.Length++;
	return ;
}




/********************************************************
函数名：  		deleteRingBuffer
作者：
日期：    	2022.10.08
功能：    	删除串口缓冲区中相应长度的数据
输入参数：		要删除的长度
返回值： 		void
修改记录：
**********************************************************/
void deleteRingBuffer(uint16_t size)
{
	if(size >= ringBuffer.Length)
	{
	    initRingBuffer();
	    return;
	}
	for(int i = 0; i < size; i++)
	{
		ringBuffer.Head = (ringBuffer.Head+1)%RINGBUFFER_LEN;//防止越界非法访问
		ringBuffer.Length--;
		return;
	}

}



/********************************************************
函数名：  		read1ByteFromRingBuffer
作者：
日期：    	2022.10.08
功能：    	从串口缓冲区读取1字节数据
输入参数：		position:读取的位置
返回值： 		所在位置的数据(1字节)
修改记录：
**********************************************************/
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
	uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;

	return ringBuffer.Ring_data[realPosition];
}




/********************************************************
函数名：  		getRingBufferLength
作者：
日期：    	2022.10.08
功能：    	获取串口缓冲区的数据数量
输入参数：
返回值： 		串口缓冲区的数据数量
修改记录：
**********************************************************/
uint16_t getRingBufferLength()
{
	return ringBuffer.Length;
}


/********************************************************
函数名：  		isRingBufferOverflow
作者：
日期：    	2022.10.08
功能：    	判断环形缓冲区是否已满
输入参数：
返回值： 		0:环形缓冲区已满 , 1:环形缓冲区未满
修改记录：
**********************************************************/
uint8_t isRingBufferOverflow()
{
	return ringBuffer.Length < RINGBUFFER_LEN;
}




void HMI_Callback(uint8_t *pData, uint16_t size)
{
		// 先把本次 DMA 收到的数据搬进环形缓冲区，下面的解析都从环形缓冲区读
		for (uint16_t i = 0; i < size; i++)
		{
			write1ByteToRingBuffer(pData[i]);
		}
		//串口数据格式：
		//串口数据帧长度：7字节
		//帧头     参数1    参数2   参数3       帧尾
		//0x55     1字节   1字节    1字节     0xffffff
		//当参数是01时
		//帧头     参数1    参数2   参数3       帧尾
		//0x55     01     led编号  led状态    0xffffff
		//例子1：上位机代码  printh 55 01 01 00 ff ff ff  含义：1号led关闭
		//例子2：上位机代码  printh 55 01 04 01 ff ff ff  含义：4号led打开
		//例子3：上位机代码  printh 55 01 00 01 ff ff ff  含义：0号led打开
		//例子4：上位机代码  printh 55 01 04 00 ff ff ff  含义：4号led关闭
		//当参数是02或03时
		//帧头     参数1    参数2   参数3       帧尾
		//0x55     02/03   滑动值    00    0xffffff
		//例子1：上位机代码  printh 55 02 64 00 ff ff ff  含义：h0.val=100
		//例子2：上位机代码  printh 55 02 00 00 ff ff ff  含义：h0.val=0
		//例子3：上位机代码  printh 55 03 64 00 ff ff ff  含义：h1.val=100
		//例子4：上位机代码  printh 55 03 00 00 ff ff ff  含义：h1.val=0
		// 当串口缓冲区大于等于一帧的长度时
		while (usize >= FRAME_LENGTH)
		{
			// 校验帧头帧尾是否匹配
			if (usize >= FRAME_LENGTH && u(0) == 0x55 && u(4) == 0xff && u(5) == 0xff && u(6) == 0xff)
			{
				// 匹配，进行解析
				if (u(1) == 0x01)
				{
					// sprintf(str, "msg.txt=\"LED %d is %s\"", u(2), u(3) ? "on" : "off");
					// tjc_send_string(str);
				} else if (u(1) == 0x02)
				{
					// // 下发的是h0进度条的信息
					// sprintf(str, "msg.txt=\"h0.val is %d\"", u(2));
					// tjc_send_string(str);
				} else if (u(1) == 0x03)
				{
					// // 下发的是h1进度条的信息
					// sprintf(str, "msg.txt=\"h1.val is %d\"", u(2));
					// tjc_send_string(str);
				}

				udelete(7); // 删除解析过的数据
			} else
			{
				// 不匹配删除1字节
				udelete(1);
				break;
			}
		}




}