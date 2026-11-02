#include "dvc_stepmotor.h"
#include "task.h"          // taskENTER_CRITICAL

/**********************************************************
*** 电机回读缓存（CAN 中断写入，任务上下文读取）
**********************************************************/
Emm_Feedback_t Emm_Feedback[EMM_MOTOR_NUM + 1];
volatile uint32_t Emm_CanRxTotal = 0;
volatile uint32_t Emm_CanRxDrop  = 0;
volatile uint32_t Emm_CanRxAll   = 0;
volatile uint32_t Emm_CanRxStd   = 0;
volatile uint32_t Emm_CanRxExt   = 0;
volatile uint32_t Emm_ReadReqTotal = 0;
volatile uint8_t  Emm_LastIdType = 0;
volatile uint32_t Emm_LastFrameId = 0;
volatile uint8_t  Emm_LastFrameDlc = 0;
volatile uint8_t  Emm_LastFrameData[8] = {0};

/**
  * @brief    读取 FDCAN 错误/队列状态（任务上下文）
  * @retval   诊断结构体
  */
Can_Diag_t Can_GetDiag(void)
{
    uint32_t ecr   = hfdcan1.Instance->ECR;
    uint32_t rxf0s = hfdcan1.Instance->RXF0S;
    uint32_t psr   = hfdcan1.Instance->PSR;
    uint32_t txfqs = hfdcan1.Instance->TXFQS;
    Can_Diag_t d;

    d.tec   = (uint8_t)(ecr & 0xFFu);
    d.rec   = (uint8_t)((ecr >> 8) & 0x7Fu);
    d.f0fl  = (uint8_t)(rxf0s & 0x7Fu);
    d.rf0l  = (uint8_t)((rxf0s >> 25) & 0x01u);
    d.psr   = (uint8_t)(psr & 0xFFu);
    d.tffl  = (uint8_t)(txfqs & 0x3Fu);
    d.state = (uint8_t)hfdcan1.State;

    return d;
}

/**
  * @brief    取电机最近一次原始位置（任务上下文）
  * @param    addr：电机地址 1~6
  * @retval   已带符号的原始位置；地址非法或还没收到过返回 0
  */
int32_t Emm_GetPosRaw(uint8_t addr)
{
    int32_t v;

    if (addr < 1u || addr > EMM_MOTOR_NUM) { return 0; }

    taskENTER_CRITICAL();
    v = Emm_Feedback[addr].pos_raw;
    taskEXIT_CRITICAL();

    return v;
}

/**********************************************************
*** 触发动作命令
**********************************************************/
/**
  * @brief    触发编码器校准
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
    static uint8_t cmd[8] = {0};

    // 装载命令（地址放ID，cmd[0]从命令码开始）
    cmd[0] =  0x06;                       // 功能码
    cmd[1] =  0x45;                       // 辅助码
    cmd[2] =  0x6B;                       // 校验字节

    // 发送命令：ID = (地址 << 8) | 包序号
    FDCAN_Transmit_Ext(&hfdcan1, ((uint32_t)addr << 8) | 0, cmd, FDCAN_DLC_BYTES_3);
}



/**********************************************************
*** 运动控制命令
**********************************************************/

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
 static uint8_t cmd[8] = {0};

    cmd[0] =  0xF3;                       // 功能码
    cmd[1] =  0xAB;                       // 辅助码
    cmd[2] =  (uint8_t)state;             // 使能状态
    cmd[3] =  snF;                        // 多机同步运动标志
    cmd[4] =  0x6B;                       // 校验字节

    FDCAN_Transmit_Ext(&hfdcan1, ((uint32_t)addr << 8), cmd, FDCAN_DLC_BYTES_5);
}



/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
 static uint8_t cmd[8] = {0};

    cmd[0] =  0xF6;                       // 功能码
    cmd[1] =  dir;                        // 方向
    cmd[2] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位
    cmd[3] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位
    cmd[4] =  acc;                        // 加速度
    cmd[5] =  snF;                        // 多机同步运动标志
    cmd[6] =  0x6B;                       // 校验字节

    // 速度控制：7字节有效
    FDCAN_Transmit_Ext(&hfdcan1, ((uint32_t)addr << 8), cmd, FDCAN_DLC_BYTES_7);
}



/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：运动标志，0为相对上一输入目标位置进行相对位置运动，1为绝对值运动，2相对当前电机实时位置进行相对位置运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF)
{
    static uint8_t cmd[16] = {0};

    // cmd[0]地址保留，FDCAN_SendCmd_Ext内部会用它构建ID
    cmd[0]  =  addr;                      // 地址（用于构建扩展ID）
    cmd[1]  =  0xFD;                      // 功能码
    cmd[2]  =  dir;                       // 方向
    cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位
    cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位
    cmd[5]  =  acc;                       // 加速度
    cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24-31)
    cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16-23)
    cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8-15)
    cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0-7)
    cmd[10] =  raF;                       // 相位/绝对标志
    cmd[11] =  snF;                       // 多机同步运动标志
    cmd[12] =  0x6B;                      // 校验字节

    // 多帧发送
    FDCAN_SendCmd_Ext(&hfdcan1, cmd, 13);
}



/**
  * @brief    角度控制（角度自动换算脉冲）
  * @param    addr ：电机地址
  * @param    dir  ：方向，0为CW，其余值为CCW
  * @param    vel  ：速度(RPM)，范围0 - 5000RPM
  * @param    acc  ：加速度，范围0 - 255
  * @param    angle：角度(°)，范围0 - 65535
  * @param    raF  ：运动标志，0相对上一目标，1绝对值，2相对当前
  * @param    snF  ：多机同步标志
  * @retval   无
  */
void Emm_V5_Angle_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint16_t angle, uint8_t raF, bool snF)
{
    // 3200脉冲 = 360° → clk = angle * 3200 / 360 = angle * 80 / 9
    uint32_t clk = (uint32_t)angle * 80 / 9;

    Emm_V5_Pos_Control(addr, dir, vel, acc, clk, raF, snF);
}



/**
  * @brief    立即停止
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    static uint8_t cmd[8] = {0};

    cmd[0] =  0xFE;                       // 功能码
    cmd[1] =  0x98;                       // 辅助码
    cmd[2] =  snF;                        // 多机同步运动标志
    cmd[3] =  0x6B;                       // 校验字节

    FDCAN_Transmit_Ext(&hfdcan1, ((uint32_t)addr << 8), cmd, FDCAN_DLC_BYTES_4);
}


/**
  * @brief    多机同步运动，触发之前带 snF=true 接收的指令同时执行
  * @param    addr  ：电机地址，0 为广播
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  static uint8_t cmd[8] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址（SendCmd内部用来构建ID）
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节

  // 发送命令：len-2=2 字节有效数据，单包发完 → [0xFF, 0x66, 0x6B]
  FDCAN_SendCmd_Ext(&hfdcan1, cmd, 4);
}




/**
  * @brief    FDCAN 接收回调（中断上下文，只做整数解析和缓存，不做浮点/阻塞）
  * @note     电机应答帧：扩展 ID = (addr << 8) | 包号，
  *           数据 = 功能码 + 返回数据 + 0x6B。
  *           0x36 读实时位置： [0x36, 符号(00正/01负), 位置(4B 大端), 0x6B]，单帧 7 字节。
  */
void ZDT_CAN_Callback(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf)
{
    /* 先计数再判断类型：标准帧也要能被发现（诊断用） */
    Emm_CanRxAll++;
    Emm_LastIdType = (uint8_t)pHeader->IdType;

    /* 原样保存一帧（标准帧也存），串口屏/调试时可以直接看到 ID/DLC/数据 */
    Emm_LastFrameId  = pHeader->Identifier;
    Emm_LastFrameDlc = (uint8_t)(pHeader->DataLength & 0x0Fu);
    for (uint8_t i = 0; i < 8u; i++)
    {
        Emm_LastFrameData[i] = (i < Emm_LastFrameDlc) ? pBuf[i] : 0u;
    }

    if (pHeader->IdType != FDCAN_EXTENDED_ID)
    {
        Emm_CanRxStd++;
        return;
    }
    Emm_CanRxExt++;

    uint32_t id   = pHeader->Identifier;
    uint8_t  addr = (uint8_t)((id >> 8) & 0xFFu);
    uint8_t  pack = (uint8_t)(id & 0xFFu);
    uint8_t  dlc  = (uint8_t)(pHeader->DataLength & 0x0Fu);

    Emm_CanRxTotal++;

    if ((addr < 1u) || (addr > EMM_MOTOR_NUM) || (pack != 0u) || (dlc < 3u) || (dlc > 8u))
    {
        Emm_CanRxDrop++;
        return;
    }

    Emm_Feedback_t *fb = &Emm_Feedback[addr];
    fb->last_code = pBuf[0];
    fb->last_dlc  = dlc;
    fb->rx_cnt++;

    if (pBuf[0] == 0x36u)                       /* 读实时位置应答 */
    {
        if ((dlc == 7u) && (pBuf[6] == 0x6Bu))
        {
            uint32_t v = ((uint32_t)pBuf[2] << 24) | ((uint32_t)pBuf[3] << 16) |
                         ((uint32_t)pBuf[4] << 8)  |  (uint32_t)pBuf[5];

            fb->pos_raw = (pBuf[1] == 0x00u) ? (int32_t)v : -(int32_t)v;
            fb->tick    = HAL_GetTick();
            fb->seq++;
            fb->valid   = 1u;
        }
        else
        {
            fb->err_cnt++;                      /* E2/EE 或长度/校验不对 */
        }
    }
    else if ((dlc == 3u) && (pBuf[1] == 0x02u))
    {
        fb->ack_cnt++;
    }
    else if ((dlc == 3u) && ((pBuf[1] == 0xE2u) || (pBuf[1] == 0xEEu)))
    {
        fb->err_cnt++;
    }
}


/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; static uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;	// 读取总线电压
		case S_CBUS : cmd[i] = 0x26; ++i; break;	// 读取总线电流
    case S_CPHA : cmd[i] = 0x27; ++i; break;	// 读取相电流
		case S_ENCO : cmd[i] = 0x29; ++i; break;	// 读取编码器原始值
		case S_CLKC : cmd[i] = 0x30; ++i; break;	// 读取实时脉冲数
    case S_ENCL : cmd[i] = 0x31; ++i; break;	// 读取经过线性化校准后的编码器值
		case S_CLKI : cmd[i] = 0x32; ++i; break;	// 读取输入脉冲数
    case S_TPOS : cmd[i] = 0x33; ++i; break;	// 读取电机目标位置
    case S_SPOS : cmd[i] = 0x34; ++i; break;	// 读取电机实时设定的目标位置
		case S_VEL  : cmd[i] = 0x35; ++i; break;	// 读取电机实时转速
    case S_CPOS : cmd[i] = 0x36; ++i; break;	// 读取电机实时位置
    case S_PERR : cmd[i] = 0x37; ++i; break;	// 读取电机位置误差
		case S_VBAT : cmd[i] = 0x38; ++i; break;	// 读取多圈编码器电池电压（Y42）
		case S_TEMP : cmd[i] = 0x39; ++i; break;	// 读取电机实时温度（Y42）
    case S_FLAG : cmd[i] = 0x3A; ++i; break;	// 读取电机状态标志位
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;	// 读取回零状态标志位
		case S_OAF  : cmd[i] = 0x3C; ++i; break;	// 读取电机状态标志位 + 回零状态标志位（Y42）
		case S_PIN  : cmd[i] = 0x3D; ++i; break;	// 读取引脚状态（Y42）
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  // 发送命令
  FDCAN_SendCmd_Ext(&hfdcan1, cmd, i);
  Emm_ReadReqTotal++;                   // 诊断：已发出的读请求数
}
