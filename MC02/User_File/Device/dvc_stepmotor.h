#ifndef DVC_STEPMOTOR_H
#define DVC_STEPMOTOR_H

#include "drv_can.h"
#include <stdbool.h>


#define					ABS(x)							((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VBUS  = 5,	// 读取总线电压
	S_CBUS  = 6,	// 读取总线电流
	S_CPHA  = 7,	// 读取相电流
	S_ENCO  = 8,	// 读取编码器原始值
	S_CLKC  = 9,	// 读取实时脉冲数
	S_ENCL  = 10,	// 读取经过线性化校准后的编码器值
	S_CLKI  = 11,	// 读取输入脉冲数
	S_TPOS  = 12,	// 读取电机目标位置
	S_SPOS  = 13,	// 读取电机实时设定的目标位置
	S_VEL   = 14,	// 读取电机实时转速
	S_CPOS  = 15,	// 读取电机实时位置
	S_PERR  = 16,	// 读取电机位置误差
	S_VBAT  = 17,	// 读取多圈编码器电池电压（Y42）
	S_TEMP  = 18,	// 读取电机实时温度（Y42）
	S_FLAG  = 19,	// 读取电机状态标志位
	S_OFLAG = 20, // 读取回零状态标志位
	S_OAF   = 21,	// 读取电机状态标志位 + 回零状态标志位（Y42）
	S_PIN   = 22,	// 读取引脚状态（Y42）
}SysParams_t;

/* 电机回读缓存：地址 1~6（底盘 1~4，升降/伸缩 5/6），下标即地址，0 不用 */
#define EMM_MOTOR_NUM  6

typedef struct
{
    volatile int32_t  pos_raw;   /* 最近一次 0x36 原始位置（已带符号，单位待上车标定） */
    volatile uint32_t tick;      /* 收到时刻 HAL_GetTick()，ms */
    volatile uint32_t seq;       /* 每收到一帧有效位置 +1 */
    volatile uint32_t rx_cnt;    /* 该地址收到的帧数 */
    volatile uint32_t ack_cnt;   /* 命令应答 02 */
    volatile uint32_t err_cnt;   /* E2/EE 或格式错误 */
    volatile uint8_t  last_code; /* 最近一帧功能码 */
    volatile uint8_t  last_dlc;
    volatile uint8_t  valid;     /* 至少收到过一帧位置 */
} Emm_Feedback_t;

extern Emm_Feedback_t Emm_Feedback[EMM_MOTOR_NUM + 1];
extern volatile uint32_t Emm_CanRxTotal;   /* 收到的扩展帧总数 */
extern volatile uint32_t Emm_CanRxDrop;    /* 地址/包号/长度不符丢掉的帧 */

/* 收发诊断：区分"没收到帧""收到标准帧""FIFO 有帧但中断没跑""总线错误" */
extern volatile uint32_t Emm_CanRxAll;     /* 所有类型的帧 */
extern volatile uint32_t Emm_CanRxStd;     /* 标准帧 */
extern volatile uint32_t Emm_CanRxExt;     /* 扩展帧 */
extern volatile uint8_t  Emm_LastIdType;   /* 最近一帧的 IdType */
extern volatile uint32_t Emm_ReadReqTotal; /* 已发出的读参数请求数 */

typedef struct
{
    uint8_t tec;    /* ECR[7:0]   发送错误计数 */
    uint8_t rec;    /* ECR[14:8]  接收错误计数 */
    uint8_t f0fl;   /* RXF0S[3:0] RX FIFO0 当前填充数 */
    uint8_t rf0l;   /* RXF0S[24]  RX FIFO0 丢帧标志 */
    uint8_t psr;    /* PSR[7:0]   LEC/ACT/EP/EW/BO */
    uint8_t tffl;   /* TXFQS[2:0] TX FIFO 空闲槽位 */
    uint8_t state;  /* HAL FDCAN 状态机 */
} Can_Diag_t;

Can_Diag_t Can_GetDiag(void);

/* 最近一帧原始数据，解析对不上时用来排查协议布局 */
extern volatile uint32_t Emm_LastFrameId;
extern volatile uint8_t  Emm_LastFrameDlc;
extern volatile uint8_t  Emm_LastFrameData[8];

/* 任务上下文取数：临界区内拷贝，避免和 CAN 中断撕裂 */
int32_t Emm_GetPosRaw(uint8_t addr);

void Emm_V5_Trig_Encoder_Cal(uint8_t addr);

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

void Emm_V5_Stop_Now(uint8_t addr, bool snF);

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);

void Emm_V5_Angle_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t angle, uint8_t raF, bool snF);

void Emm_V5_Synchronous_motion(uint8_t addr);

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

void ZDT_CAN_Callback(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf);

#endif // DVC_STEPMOTOR_H   