#ifndef __DVC_YIS320_H__
#define __DVC_YIS320_H__
#include "main.h"

#define YIS_HEADER_1     0x59
#define YIS_HEADER_2     0x53
#define YIS_PACKET_MAX_LEN  (2 + 2 + 1 + 255 + 2) // 最大数据包长度
#define YIS_HEADER_SIZE  2
#define YIS_TID_SIZE     2
#define YIS_LEN_SIZE     1
#define YIS_CHECKSUM_SIZE 2

// 数据包解析状态机
typedef enum {
  YIS_STATE_HEADER_1 = 0,
  YIS_STATE_HEADER_2,
  YIS_STATE_TID,
  YIS_STATE_LEN,
  YIS_STATE_DATA,
  YIS_STATE_CHECKSUM
} YIS_ParseState_t;

// 数据包结构体
typedef struct {
  uint8_t buffer[YIS_PACKET_MAX_LEN];
  uint16_t tid;
  uint8_t data_len;
  uint8_t data[255];
  uint8_t ck1, ck2;
  uint16_t index;
  YIS_ParseState_t state;
} YIS_packet_t;

// 数据解析结构体
typedef struct {
    float accel[3];     // 加速度 (m/s²)
    float euler[3];     // 欧拉角 (度): pitch, roll, yaw
    uint16_t tid;       // 帧序号
    uint8_t data_len;   // 数据域长度
} YIS_Data_t;

/* 接收链路：IMU_RxStart() 在 main() 里调度器启动前调用，
 * 之后每个字节由 USART10 中断走 IMU_RxCpltISR() 喂进解析器。 */
void  IMU_init(void);
void  IMU_RxStart(void);
void  IMU_RxCpltISR(void);
void  IMU_OnUartError(uint32_t error_code);
void  IMU_RxWatchdog(void);

float IMU_getYaw(void);
void  IMU_getAcc(float returnData[3]);
void  IMU_resetYaw(void);
void  IMU_prob(void);

/* 调试计数，供 LCD / 串口屏直接显示 */
extern volatile uint32_t imu_byte_cnt;   // 收到的原始字节数
extern volatile uint32_t imu_pkt_cnt;    // 通过校验的包数
extern volatile uint32_t imu_cs_err;     // 校验和失败次数
extern volatile uint32_t imu_err_cnt;    // UART 硬件错误次数
extern volatile uint32_t imu_last_err;   // 最近一次 ErrorCode
extern volatile uint8_t  imu_last_raw[40];  // 最近一次完整帧的原始字节
extern volatile uint8_t  imu_last_raw_len;

#endif