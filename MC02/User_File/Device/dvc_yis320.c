
#include "dvc_yis320.h"
#include "usart.h"

/* HAL 单字节接收缓冲，地址必须常驻 */
static uint8_t s_yis_rx_byte;

static float inital_deg = 0.0f;    // 零位偏置（度）
static float raw_yaw = 0.0f;       // 未经偏置的原始yaw（度），每次有yaw数据时更新
static uint8_t yaw_ready = 0;      // 是否已收到过有效yaw数据
static YIS_Data_t yis_data;

// debug
volatile uint32_t imu_byte_cnt = 0;    // 收到的原始字节数
volatile uint32_t imu_pkt_cnt = 0;     // 通过校验的包数
volatile uint32_t imu_cs_err = 0;      // 校验和失败次数
volatile uint32_t imu_err_cnt = 0;     // UART 硬件错误次数
volatile uint32_t imu_last_err = 0;    // 最近一次 ErrorCode
volatile uint8_t  imu_last_raw[40];    // 最近一次完整帧的原始字节
volatile uint8_t  imu_last_raw_len = 0;
volatile uint8_t  imu_rx_ring[64];     // 无条件记录最近64个收到的字节
volatile uint8_t  imu_rx_ring_idx = 0;

static const uint8_t yis_bias_calib_cmd[9] = {
    0x59, 0x53, 0x4D, 0x12, 0x00, 0x50, 0x01, 0xB0, 0x6A
};


// 计算校验和
static void YIS_calculateChecksum(uint8_t *data, uint16_t len, uint8_t *ck1, uint8_t *ck2) {
  uint16_t i;
  *ck1 = 0;
  *ck2 = 0;
  for (i = 0; i < len; i++) {
    *ck1 += data[i];
    *ck2 += *ck1;
  }
}

/* 按小端读取 int32。全程用无符号拼接再转有符号，
 * 避免 (0xFF << 24) 这种落在 int 上的有符号左移（实现定义行为）。 */
static int32_t YIS_readInt32LE(const uint8_t *p)
{
  uint32_t v = (uint32_t)p[0]
             | ((uint32_t)p[1] << 8)
             | ((uint32_t)p[2] << 16)
             | ((uint32_t)p[3] << 24);
  return (int32_t)v;
}

static YIS_packet_t yis_packet; 

// 重置数据包解析状态
static void YIS_resetPacket(void) {
  memset(&yis_packet, 0, sizeof(yis_packet));
  yis_packet.state = YIS_STATE_HEADER_1;
}


// 数据包处理函数
static void YIS_processPacket(YIS_packet_t *packet) {
    // 提取TID（小端模式）
    packet->tid = (packet->buffer[3] << 8) | packet->buffer[2];
    yis_data.tid = packet->tid;
    yis_data.data_len = packet->data_len;
    // 提取数据域
    memcpy(packet->data, &packet->buffer[YIS_HEADER_SIZE + YIS_TID_SIZE + YIS_LEN_SIZE], packet->data_len);
    // 解析数据域中的各个数据包
    uint16_t data_index = 0;
    /* 至少要有 data_id + data_packet_len 两个字节，再决定要不要继续 */
    while (data_index + 2 <= packet->data_len) {
        uint8_t data_id = packet->data[data_index++];
        uint8_t data_packet_len = packet->data[data_index++];
        // 检查数据长度是否有效
        if (data_index + data_packet_len > packet->data_len) {
            break;
        }
        // 根据DATA ID解析具体数据
        switch (data_id) {
            case 0x10: // 加速度数据
                if (data_packet_len == 12) {
                    yis_data.accel[0] = YIS_readInt32LE(&packet->data[data_index])     * 0.000001f; // 转换为m/s²
                    yis_data.accel[1] = YIS_readInt32LE(&packet->data[data_index + 4]) * 0.000001f;
                    yis_data.accel[2] = YIS_readInt32LE(&packet->data[data_index + 8]) * 0.000001f;
                }
                break;

            case 0x40: // 欧拉角数据
                if (data_packet_len == 12) {
                    // 俯仰角Pitch
                    yis_data.euler[0] = YIS_readInt32LE(&packet->data[data_index]) * 0.000001f; // 转换为度
                    // 横滚角Roll
                    yis_data.euler[1] = YIS_readInt32LE(&packet->data[data_index + 4]) * 0.000001f;
                    // 航向角Yaw
                    raw_yaw = YIS_readInt32LE(&packet->data[data_index + 8]) * 0.000001f;  // 保存原始绝对值（度）
                    // 自动设零：首次收到yaw数据时以当前值为零位
                    if(!yaw_ready) {
                        inital_deg = raw_yaw;
                        yaw_ready = 1;
                    }
                    // 减去零位偏置，得到相对角度
                    yis_data.euler[2] = raw_yaw - inital_deg;
                    if(yis_data.euler[2] < -180) yis_data.euler[2] += 360;
                    if(yis_data.euler[2] > 180) yis_data.euler[2] -= 360;
                }
                break;
                
            // 其他DATA ID忽略
            default:
                break;
        }
        // 移动到下一个数据包
        data_index += data_packet_len;
    }
}

static void IMU_UARTReceive(uint8_t buffer){
    imu_byte_cnt++;
    imu_rx_ring[imu_rx_ring_idx] = buffer;
    imu_rx_ring_idx = (imu_rx_ring_idx + 1) & 0x3F;
    switch (yis_packet.state) {
      case YIS_STATE_HEADER_1:
        if (buffer == YIS_HEADER_1) {
          yis_packet.buffer[yis_packet.index++] = buffer;
          yis_packet.state = YIS_STATE_HEADER_2;
        } else {
          YIS_resetPacket(); // 重置状态机
        }
        break;

      case YIS_STATE_HEADER_2:
        if (buffer == YIS_HEADER_2) {
          yis_packet.buffer[yis_packet.index++] = buffer;
          yis_packet.state = YIS_STATE_TID;
        } else {
          YIS_resetPacket();
        }
        break;

      case YIS_STATE_TID:
        yis_packet.buffer[yis_packet.index++] = buffer;
        if (yis_packet.index == YIS_HEADER_SIZE + YIS_TID_SIZE) {
          yis_packet.state = YIS_STATE_LEN;
        }
        break;

      case YIS_STATE_LEN:
        yis_packet.data_len = buffer;
        yis_packet.buffer[yis_packet.index++] = buffer;
        if (yis_packet.data_len == 0) {
          yis_packet.state = YIS_STATE_CHECKSUM;
        } else {
          yis_packet.state = YIS_STATE_DATA;
        }
        break;

      case YIS_STATE_DATA:
        yis_packet.buffer[yis_packet.index++] = buffer;
        if (yis_packet.index == YIS_HEADER_SIZE + YIS_TID_SIZE + YIS_LEN_SIZE + yis_packet.data_len) {
          yis_packet.state = YIS_STATE_CHECKSUM;
        }
        break;

      case YIS_STATE_CHECKSUM:
        yis_packet.buffer[yis_packet.index++] = buffer;
        if (yis_packet.index == YIS_HEADER_SIZE + YIS_TID_SIZE + YIS_LEN_SIZE + yis_packet.data_len + YIS_CHECKSUM_SIZE) {
          // 保存原始帧用于调试
          imu_last_raw_len = yis_packet.index;
          memcpy((void*)imu_last_raw, yis_packet.buffer, imu_last_raw_len > 40 ? 40 : imu_last_raw_len);
          // 验证校验和
          uint8_t calc_ck1, calc_ck2;
          YIS_calculateChecksum(&yis_packet.buffer[YIS_HEADER_SIZE],
                               YIS_TID_SIZE + YIS_LEN_SIZE + yis_packet.data_len,
                               &calc_ck1, &calc_ck2);

          if (calc_ck1 == yis_packet.buffer[yis_packet.index - 2] &&
              calc_ck2 == yis_packet.buffer[yis_packet.index - 1]) {
            imu_pkt_cnt++;
            YIS_processPacket(&yis_packet);
          } else {
            imu_cs_err++;
          }
          YIS_resetPacket(); // 重置状态机准备接收下一个数据包
        }
        break;

      default:
        YIS_resetPacket();
        break;
    }
}

void IMU_init(void){
    YIS_resetPacket();
}

/* 启动 USART10 单字节中断接收。可在调度器启动前调用。 */
void IMU_RxStart(void)
{
    (void)HAL_UART_Receive_IT(&huart10, &s_yis_rx_byte, 1);
}

/* 只在 HAL_UART_RxCpltCallback 里调用（中断上下文）。 */
void IMU_RxCpltISR(void)
{
    IMU_UARTReceive(s_yis_rx_byte);
    IMU_RxStart();                 /* HAL 每收 1 字节就停，必须重挂 */
}

void IMU_OnUartError(uint32_t error_code)
{
    imu_last_err = error_code;
    imu_err_cnt++;
    /* ORE 后 HAL 已把 RxState 置回 READY，此时重挂安全；
       FE/NE 不停止接收，重挂只会拿到 HAL_BUSY，先判断状态再动。 */
    if (huart10.RxState == HAL_UART_STATE_READY)
    {
        IMU_RxStart();
    }
}

/* 兜底：防止某次错误后接收静默停住。由低频任务调用，不要求实时。 */
void IMU_RxWatchdog(void)
{
    if (huart10.RxState == HAL_UART_STATE_READY)
    {
        IMU_RxStart();
    }
}

/**
 * @brief 用于获取IMU的yaw轴解算结果。每次reset时，坐标会被重置为0度。
 *
 * @return float yaw轴输出（度），范围 [-180, 180)
 */
float IMU_getYaw(void){
    return yis_data.euler[2];
}

/**
 * @brief 将当前车头方向设为yaw的零度。
 *        调用者需确保IMU已经收到过有效yaw数据（上电后约100ms即可）。
 */
/* 和 USART10 中断里对 inital_deg/raw_yaw/yaw_ready 的写入互斥 */
void IMU_resetYaw(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    inital_deg = raw_yaw;
    yaw_ready = 1;
    yis_data.euler[2] = 0.0f;
    __set_PRIMASK(primask);
}

/**
 * @brief 
 * 
 * @param returnData 
 */
void IMU_getAcc(float returnData[3]){
  returnData[0] = yis_data.accel[0];
  returnData[1] = yis_data.accel[1];
  returnData[2] = yis_data.accel[2];
}

void IMU_prob(void) {
    volatile int i;
    for(i = 0; i < imu_last_raw_len; i++) imu_last_raw[i];  // 断点看这里
    (void)i;
}