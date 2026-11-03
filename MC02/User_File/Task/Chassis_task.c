#include "Chassis_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "alg_fastmath.h"
#include "IMU_task.h"
#include "dvc_yis320.h"
#include "task.h"          // taskENTER_CRITICAL / taskEXIT_CRITICAL
#include <math.h>
#include <stdint.h>

/** 
        底盘移动坐标系    W方向遵循坐标系右手法则
        ^y
        |                   
        |______>x          


        底盘俯视图
        
速度解算俯视图
 Speed[1]          Speed[2]                  
    \               /
      \           /
        \       /
          \   /
            *
          /    \
        /        \
      /            \
    /                \
Speed[3]           Speed[4]


        
实际底盘电机ID俯视图
 0x04               0x03                |
    \               /                   |
      \           /                     |
        \       /   {  }                   
          \   /    {tower}{claw}
            *       {  }
          /    \
        /        \
      /            \
    /     {tray}     \
0x01                0x02

*/

uint8_t acc_vel=230;
/* 位置运动加速度：0=直接启动(瞬时)，非 0 使用加速度曲线（直线起步用，偏柔和） */
uint8_t acc_pos=125;
/* 原地旋转加速度：0=直接启动，最快（旋转要快速到位，不共用直线的柔和值） */
uint8_t acc_rot=0;
int16_t  car_move_speed = 50;

chassis_t chassis = {0};   // 头文件里只有 extern 声明，定义放在这里

/********************** 里程计 ************************/
/* 实测：前进 100mm 四轮计数变化约 26071，理论 100/(π×80)×65536 = 26077.9 → 65536 计数/圈 */
#define ODOM_COUNTS_PER_REV     65536.0f
#define ODOM_MM_PER_COUNT       (3.14159265f * CHASSIS_WHEEL_DIAMETER_MM / ODOM_COUNTS_PER_REV)
#define ODOM_STALE_MS           80u       /* 20ms 轮询，允许连丢 3~4 帧 */
#define ODOM_SETTLE_N           3u        /* 连续 3 个周期都到位才判完成 */
#define CHASSIS_POS_TOL_MM      2.0f      /* 轮子行程到位容差 */
#define CHASSIS_POS_MIN_MM      5.0f      /* 行程太小时容差没有意义，交给超时判定 */
#define CHASSIS_POS_TIMEOUT_MS  300u      /* 位置判定失败时的硬超时余量 */

/* 逻辑轮顺序与 Chassis_Absolute_Pos_Ctrl_Line 的 s[4] 一致：{addr4, addr3, addr1, addr2} */
static const uint8_t k_odom_addr[4] = {4, 3, 1, 2};
/* 前进实测：addr4/addr1 计数增大，addr3/addr2 减小；逻辑行程取正需要翻符号 */
static const int8_t  k_odom_sign[4] = {+1, -1, +1, -1};

static int32_t  s_last_raw[4];      /* 上一次原始位置 */
static uint32_t s_last_seq[4];      /* 上一次处理的反馈序号 */
static float    s_wheel_mm[4];      /* 逻辑轮累计行程，mm，带符号 */
static float    s_psi_prev;         /* 上一周期航向，rad */
static uint8_t  s_odom_seeded;      /* 已完成首轮基准 */
static uint8_t  s_odom_valid;       /* 四轮都新鲜，本次位姿有效 */
static uint8_t  s_line_settle;      /* 直线到位连续计数 */
static uint8_t  s_rot_settle;       /* 旋转到位连续计数 */

static void Chassis_PollEncoder(void);
static void Chassis_PrimeOdometry(void);

void Chassis_Init(void)
{
    AttachInterrupt_FDCAN(&hfdcan1, ZDT_CAN_Callback);
    // Emm_V5_En_Control(2, true, true);                 // 使能电机
    osDelay(500);


  /* 底盘x坐标pid初始化 */
    PID_Init(&chassis.PID_x, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
    /* 底盘y坐标pid初始化 */
    PID_Init(&chassis.PID_y, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
    /* 底盘角度pid：err 单位rad，输出单位RPM。起步值，需上车标定 */
    PID_Init(&chassis.PID_angle, 60.0f, 2.0f, 5.0f, 50.0f, 20.0f);
    /* 旋转航向pid初始化：err 单位rad，输出单位近似RPM。以下为起步值，需上车标定 */
    PID_Init(&chassis.PID_rotate, 100.0f, 0.5f, 5.0f, 500.0f, 100.0f);

    /* 等四轮首帧并建立里程计基准，保证第一段动作的行程不被 seed 吃掉 */
    Chassis_PrimeOdometry();
}


void Chassis_Task(void *argument)
{
    (void)argument;
    Chassis_Init();
    // Chassis_Absolute_Pos_Ctrl_Line(0, 2000, 30);   /* 上电测试：前进 2m / 3s，验证完请删除 */
    // Chassis_Rotate(90, 100);   /* 上电测试：顺时针旋转 90° / 50RPM，验证完请删除 */

    //二维码区
//    Chassis_Absolute_Pos_Ctrl_Line(0, 1075, 30);   /* 上电测试：后退 1075mm / 3s，验证完请删除 */
//    Chassis_Relative_Pos_Ctrl_Line(-1075, 1075, 30);   /* 上电测试：后退 1075mm / 3s，验证完请删除 */


    for (;;)
    {
        Chassis_Run();
        Chassis_PollEncoder();
        osDelay(10);               // 速度模式调试时改回 Chassis_Move(...)
    }
}



/* 轮转轮询电机实时位置：每周期读 2 个，四个轮子各 20ms 更新一次。
   TX FIFO 只有 3 层，必须逐帧间隔发送；FIFO 满时跳过本轮，避免丢帧计数。 */
static void Chassis_PollEncoder(void)
{
    static const uint8_t k_wheel_addr[4] = {4, 3, 1, 2};   /* 与逆解数组索引一一对应 */
    static uint8_t idx = 0;

    for (uint8_t i = 0; i < 2u; i++)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0u)
        {
            Emm_V5_Read_Sys_Params(k_wheel_addr[idx], S_CPOS);
        }
        idx = (idx + 1u) & 3u;
        osDelay(1);
    }
}


void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W)
{
    int16_t speed[4];

    speed[0] = +Vy +Vx -W;
	speed[1] = -Vy + Vx +W;
	speed[2] = -Vy + Vx -W;
	speed[3] = +Vy + Vx +W;

    Chassis_SetMotor_Vel(speed);
}


void Chassis_SetMotor_Pos(int16_t speed[4],uint32_t clk)
{
    //正向
    if(speed[0]>=0){Emm_V5_Pos_Control(4, 0, speed[0], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(4, 1, -speed[0], acc_pos, clk, 0, true);}
	osDelay(1);
    //正向
	if(speed[1]>=0){Emm_V5_Pos_Control(3, 1, speed[1], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(3, 0, -speed[1], acc_pos, clk, 0, true);}
	osDelay(1);
    //正向
	if(speed[2]>=0){Emm_V5_Pos_Control(1, 0, speed[2], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(1, 1, -speed[2], acc_pos, clk, 0, true);}
	osDelay(1);
    //正向
	if(speed[3]>=0){Emm_V5_Pos_Control(2, 1, speed[3], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(2, 0, -speed[3], acc_pos, clk, 0, true);}
    osDelay(1);
    Emm_V5_Synchronous_motion(0); //广播触发
}


void Chassis_SetMotor_Vel(int16_t speed[4])//此处用4，方便与电机对应1，2，3，4
{
    //正向
    if(speed[0]>=0){Emm_V5_Vel_Control(4, 0, speed[0], acc_vel, true);}else{Emm_V5_Vel_Control(4, 1, -speed[0], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[1]>=0){Emm_V5_Vel_Control(3, 0, speed[1], acc_vel, true);}else{Emm_V5_Vel_Control(3, 1, -speed[1], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[2]>=0){Emm_V5_Vel_Control(1, 1, speed[2], acc_vel, true);}else{Emm_V5_Vel_Control(1, 0, -speed[2], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[3]>=0){Emm_V5_Vel_Control(2, 1, speed[3], acc_vel, true);}else{Emm_V5_Vel_Control(2, 0, -speed[3], acc_vel, true);}
    osDelay(1);
    Emm_V5_Synchronous_motion(0); //广播触发
}


/* 角度归一化到 (-PI, PI]，避免 yaw 在 ±180° 处跳变把误差放大成 2π */
static float Chassis_WrapPi(float a)
{
    while (a > PI)  { a -= 2.0f * PI; }
    while (a < -PI) { a += 2.0f * PI; }
    return a;
}

/* 底盘航向来源：YIS320 的 yaw 是度、范围 [-180,180)，转成 rad 与 setYaw 同量纲 */
static inline float Chassis_GetYawRad(void)
{
    return IMU_getYaw() * DEG_TO_RAD;
}

/* 临界区内取四轮反馈快照，保证 pos/seq/tick 来自同一次更新 */
static void Chassis_ReadOdomSnapshot(int32_t raw[4], uint32_t seq[4], uint32_t tick[4])
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < 4u; i++)
    {
        raw[i]  = Emm_Feedback[k_odom_addr[i]].pos_raw;
        seq[i]  = Emm_Feedback[k_odom_addr[i]].seq;
        tick[i] = Emm_Feedback[k_odom_addr[i]].tick;
    }
    taskEXIT_CRITICAL();
}

/* 上电时先轮询到四轮首帧，把里程计基准建立在动车之前，避免第一段行程被 seed 吃掉 */
static void Chassis_PrimeOdometry(void)
{
    for (uint8_t n = 0; n < 50u; n++)
    {
        Chassis_PollEncoder();
        if (Emm_Feedback[1].valid && Emm_Feedback[2].valid &&
            Emm_Feedback[3].valid && Emm_Feedback[4].valid)
        {
            break;
        }
        osDelay(10);
    }

    int32_t  raw[4];
    uint32_t seq[4];
    uint32_t tick[4];

    /* 四轮都拿到过帧才建立基准；否则留给 UpdateOdometry 的首轮 seed，
       避免用 0 当基准算出一个巨大的假位移 */
    if (!(Emm_Feedback[1].valid && Emm_Feedback[2].valid &&
          Emm_Feedback[3].valid && Emm_Feedback[4].valid))
    {
        s_odom_seeded = 0;
        return;
    }

    Chassis_ReadOdomSnapshot(raw, seq, tick);
    for (uint8_t i = 0; i < 4u; i++)
    {
        s_last_raw[i] = raw[i];
        s_last_seq[i] = seq[i];
    }
    s_psi_prev    = Chassis_GetYawRad();
    s_odom_seeded = 1;
    s_odom_valid  = 0;      /* 等第一轮新帧到达后再置 1 */
}

/* 里程计：四轮都拿到新帧时做一次差分 + 正运动学 + 世界系积分（50Hz）。
   约定与 Chassis_Absolute_Pos_Ctrl_Line 的逆解严格互逆：
   s0=+bx+by(addr4), s1=-bx+by(addr3), s2=-bx+by(addr1), s3=+bx+by(addr2) */
static void Chassis_UpdateOdometry(float yaw_now)
{
    int32_t  raw[4];
    uint32_t seq[4];
    uint32_t tick[4];
    uint32_t now = HAL_GetTick();
    uint8_t  stale = 0;

    Chassis_ReadOdomSnapshot(raw, seq, tick);

    /* 只有四轮都更新过才积分，避免两两采样时刻错开污染 bx/by */
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (seq[i] == s_last_seq[i]) { return; }
    }

    for (uint8_t i = 0; i < 4u; i++)
    {
        if ((now - tick[i]) > ODOM_STALE_MS) { stale = 1; }
    }

    if ((!s_odom_seeded) || stale)
    {
        /* 首次或任一轮掉线：重新取基准，丢弃停机期间的未知行程，不产生跳变 */
        for (uint8_t i = 0; i < 4u; i++)
        {
            s_last_raw[i] = raw[i];
            s_last_seq[i] = seq[i];
        }
        s_psi_prev    = yaw_now;
        s_odom_seeded = 1;
        s_odom_valid  = 0;
        return;
    }

    float d[4];
    for (uint8_t i = 0; i < 4u; i++)
    {
        int32_t delta = raw[i] - s_last_raw[i];
        /* 单圈展开：兼容单圈 0~65535 回绕和多圈累计，单次移动必须小于半圈 */
        if (delta >  32768) { delta -= 65536; }
        if (delta < -32768) { delta += 65536; }

        s_last_raw[i] = raw[i];
        s_last_seq[i] = seq[i];

        d[i] = (float)k_odom_sign[i] * (float)delta * ODOM_MM_PER_COUNT;
        s_wheel_mm[i] += d[i];
    }

    float bx = (d[0] - d[1] - d[2] + d[3]) * 0.25f;
    float by = (d[0] + d[1] + d[2] + d[3]) * 0.25f;

    /* yaw 中点积分到世界系，约定与 Chassis_OutputVel 的 atan2(x,y)+yaw 一致 */
    float dpsi    = Chassis_WrapPi(yaw_now - s_psi_prev);
    float psi_mid = yaw_now - 0.5f * dpsi;
    s_psi_prev = yaw_now;

    chassis.Px += bx * cosf(psi_mid) - by * sinf(psi_mid);
    chassis.Py += bx * sinf(psi_mid) + by * cosf(psi_mid);

    s_odom_valid = 1;
}

/* 本次直线/旋转是否已到位：四轮实测行程与目标行程之差都在容差内 */
static uint8_t Chassis_LineArrived(void)
{
    float max_need = 0.0f;

    if (!s_odom_valid) { return 0; }

    for (uint8_t i = 0; i < 4u; i++)
    {
        float need = fabsf(chassis.line.s_mm[i]);
        if (need > max_need) { max_need = need; }
    }
    if (max_need < CHASSIS_POS_MIN_MM) { return 0; }   /* 小动作交给超时判定 */

    for (uint8_t i = 0; i < 4u; i++)
    {
        float prog = s_wheel_mm[i] - chassis.line.start_mm[i];
        if (fabsf(chassis.line.s_mm[i] - prog) > CHASSIS_POS_TOL_MM) { return 0; }
    }
    return 1;
}

static uint8_t Chassis_RotateArrived(void)
{
    float max_need = 0.0f;

    if (!s_odom_valid) { return 0; }

    for (uint8_t i = 0; i < 4u; i++)
    {
        float need = fabsf(chassis.rotate_s_mm[i]);
        if (need > max_need) { max_need = need; }
    }
    if (max_need < CHASSIS_POS_MIN_MM) { return 0; }   /* 小动作交给超时判定 */

    for (uint8_t i = 0; i < 4u; i++)
    {
        float prog = s_wheel_mm[i] - chassis.rotate_start_mm[i];
        if (fabsf(chassis.rotate_s_mm[i] - prog) > CHASSIS_POS_TOL_MM) { return 0; }
    }
    return 1;
}

/* 浮点 RPM 转驱动器可接受的整数 RPM：四舍五入并钳位到 ±5000（驱动器范围 0~5000，符号单独下发） */
static int16_t Chassis_ClampRpm(float v)
{
    if (v >  5000.0f) v =  5000.0f;
    if (v < -5000.0f) v = -5000.0f;
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* 世界系速度矢量(轮 RPM 域) + 角速度 W -> 四轮 RPM 下发。
   与 Chassis_Move 同一套符号，不要在此改符号。 */
static void Chassis_OutputVel(float vx_cmd, float vy_cmd, float w, float yaw_now)
{
    /* 合速度大小 */
    float V = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    /* 合速度与Y轴夹角 */
    float angle = atan2f(vx_cmd, vy_cmd) + yaw_now;
    /* Vx */
    float Vx = V * fast_sin(angle);
    /* Vy */
    float Vy = V * fast_cos(angle);

    chassis.Motors_Speed[0] = Chassis_ClampRpm(Vx + Vy - w);
    chassis.Motors_Speed[1] = Chassis_ClampRpm(Vx - Vy + w);
    chassis.Motors_Speed[2] = Chassis_ClampRpm(Vx - Vy - w);
    chassis.Motors_Speed[3] = Chassis_ClampRpm(Vx + Vy + w);

    Chassis_SetMotor_Vel(chassis.Motors_Speed);
}

/* 指定每轮速度下发位置指令（raF=0）。acc 为该次运动的加速度(0=直接启动)。
   方向映射与 Chassis_SetMotor_Pos 一致：4→正0/负1，3→正1/负0，1→正0/负1，2→正1/负0 */
static void Chassis_SendWheelPosVel(const float s_mm[4], const uint16_t vel[4], uint8_t acc)
{
    float    pulses[4];
    uint16_t v[4];
    uint32_t clk[4];

    for (int i = 0; i < 4; i++)
    {
        pulses[i] = s_mm[i] * CHASSIS_PULSE_PER_MM;
        clk[i]    = (uint32_t)(fabsf(pulses[i]) + 0.5f);
        v[i]      = (vel[i] > 0u) ? vel[i] : 1u;
    }

    if (clk[0]) { Emm_V5_Pos_Control(4, pulses[0] >= 0.0f ? 0 : 1, v[0], acc, clk[0], 0, true); }
    osDelay(1);
    if (clk[1]) { Emm_V5_Pos_Control(3, pulses[1] >= 0.0f ? 1 : 0, v[1], acc, clk[1], 0, true); }
    osDelay(1);
    if (clk[2]) { Emm_V5_Pos_Control(1, pulses[2] >= 0.0f ? 0 : 1, v[2], acc, clk[2], 0, true); }
    osDelay(1);
    if (clk[3]) { Emm_V5_Pos_Control(2, pulses[3] >= 0.0f ? 1 : 0, v[3], acc, clk[3], 0, true); }
    osDelay(1);

    Emm_V5_Synchronous_motion(0);
}

/* 按行程和时长下发：四轮速度按脉冲比例分配 → 同时到达。acc 为加速度(0=直接启动)。
   返回预计运动时长(ms)，供完成判定使用。 */
static uint32_t Chassis_SendWheelPos(const float s_mm[4], float time_s, uint8_t acc)
{
    float pulses[4];
    float vel_rpm[4];
    float max_rpm = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        pulses[i]  = s_mm[i] * CHASSIS_PULSE_PER_MM;
        vel_rpm[i] = fabsf(pulses[i]) / CHASSIS_PULSE_PER_REV * 60.0f / time_s;
        if (vel_rpm[i] > max_rpm) max_rpm = vel_rpm[i];
    }

    /* 超过驱动器上限时整体等比缩小，保持四轮比例（耗时略增，但不会走歪） */
    float scale = (max_rpm > 5000.0f) ? (5000.0f / max_rpm) : 1.0f;

    uint16_t vel[4];
    float    dur_s = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        vel_rpm[i] *= scale;
        uint32_t clk = (uint32_t)(fabsf(pulses[i]) + 0.5f);
        vel[i] = (uint16_t)(vel_rpm[i] + 0.5f);
        if (clk == 0u) { vel[i] = 0u; continue; }
        if (vel[i] < 1u) vel[i] = 1u;

        float d = (float)clk / CHASSIS_PULSE_PER_REV * 60.0f / (float)vel[i];
        if (d > dur_s) dur_s = d;
    }

    Chassis_SendWheelPosVel(s_mm, vel, acc);

    return (uint32_t)(dur_s * 1000.0f + 0.5f);
}

/* 按相对角度(rad)原地旋转。用驱动器位置环执行：
   把旋转角换算成四轮各自走过的弧长(脉冲数)，四轮同时到达 → 绕中心转动，位置基本不变；
   到位后驱动器闭环保位，不需要航向 PID。
   dtheta: 需要转过的相对角度(rad)，逆时针为正
   speed : 轮子转速, 单位RPM
   非阻塞：下发后立即返回，完成由 Chassis_Run 的 MOTOR_POS_MODE 判定。 */
static void Chassis_RotateByRad(float dtheta, uint8_t speed)
{
    dtheta = Chassis_WrapPi(dtheta);

    /* 轮子沿滚动方向的弧长(mm)，CHASSIS_ROTATE_ARM_MM 为实测标定的旋转力臂 */
    float arc = dtheta * CHASSIS_ROTATE_ARM_MM;
    if (fabsf(arc) < 1.0f)
    {
        chassis.rotate_done = 1;
        return;
    }

    /* 旋转列：addr4/addr1 反向，addr3/addr2 正向（与 Chassis_Move 的 W 列一致） */
    float s[4];
    s[0] = -arc;   /* addr 4 */
    s[1] = +arc;   /* addr 3 */
    s[2] = -arc;   /* addr 1 */
    s[3] = +arc;   /* addr 2 */

    /* speed 解释为轮子转速(RPM)，据此估算时长 */
    float wheel_mm_s = (float)speed / 60.0f * (3.14159265f * CHASSIS_WHEEL_DIAMETER_MM);
    if (wheel_mm_s < 1.0f) wheel_mm_s = 1.0f;
    float time_s = fabsf(arc) / wheel_mm_s;

    taskENTER_CRITICAL();
    chassis.setVx = 0.0f;
    chassis.setVy = 0.0f;
    chassis.setVw = 0.0f;
    chassis.setYaw = Chassis_WrapPi(Chassis_GetYawRad() + dtheta);  /* 供后续速度模式航向保持使用 */
    chassis.rotate_done = 0;
    chassis.rotate_active = 1;
    chassis.rotate_start_tick = osKernelGetTickCount();
    chassis.rotate_total_ms   = (uint32_t)(time_s * 1000.0f + 0.5f);
    for (uint8_t i = 0; i < 4u; i++)
    {
        chassis.rotate_s_mm[i]     = s[i];
        chassis.rotate_start_mm[i] = s_wheel_mm[i];
    }
    s_rot_settle = 0;
    chassis.ctrlMode = MOTOR_POS_MODE;
    taskEXIT_CRITICAL();

    /* 下发四轮位置指令（内部有 osDelay），下发后再起计时 */
    uint32_t dur = Chassis_SendWheelPos(s, time_s, acc_rot);
    taskENTER_CRITICAL();
    chassis.rotate_start_tick = osKernelGetTickCount();
    chassis.rotate_total_ms   = dur ? dur : (uint32_t)(time_s * 1000.0f);
    taskEXIT_CRITICAL();
}

/* 原地旋转到绝对目标航向。 */
void Chassis_Rotate(int16_t angle, uint8_t speed)
{
    if (angle < -180 || angle > 180) return;
    if (chassis.rotate_active || chassis.line.active) return;  /* 对应旧版 flag.car_in_task == 0 封锁 */
    Chassis_RotateByRad((float)angle * DEG_TO_RAD - Chassis_GetYawRad(), speed);
}





/* 直线移动内核（绝对/相对共用）。位移由 Emm_V5 位置环执行：
   四轮按逆解分配脉冲数、按比例分配速度、同时到达 → 直行且航向不变，
   到位后驱动器闭环保位，不会漂。
   x/y: mm。relative=0 时是绝对目标坐标；relative=1 时是相对当前指令坐标的世界系位移
        （+y 前进、+x 横移）。
   time_ds: 期望总时长，单位 0.1s（0.1~25.5s），决定运动速度
   返回 1=已受理，0=被拒绝。只能在任务上下文调用，不要在中断里调。 */
static uint8_t Chassis_LineMove(float x, float y, uint8_t time_ds, uint8_t relative)
{
    if (time_ds == 0u) { return 0; }

    /* 检查并置忙必须原子，避免两个任务同时通过守卫；
       相对移动的目标依赖 last_x/last_y，也必须在同一临界区内计算 */
    taskENTER_CRITICAL();
    if (chassis.line.active || chassis.rotate_active)
    {
        taskEXIT_CRITICAL();
        return 0;
    }

    float target_x = relative ? (chassis.line.last_x + x) : x;
    float target_y = relative ? (chassis.line.last_y + y) : y;

    float dx = target_x - chassis.line.last_x;
    float dy = target_y - chassis.line.last_y;
    float distance = sqrtf(dx * dx + dy * dy);
    if ((distance < CHASSIS_LINE_MIN_DIST) || (distance > CHASSIS_LINE_MAX_DIST_MM))
    {
        taskEXIT_CRITICAL();
        return 0;
    }

    /* 世界系位移 -> 机体系分量：与 Chassis_OutputVel 同一 +yaw 旋转约定 */
    float yaw   = Chassis_GetYawRad();
    float angle = atan2f(dx, dy) + yaw;
    float bx    = distance * fast_sin(angle);
    float by    = distance * fast_cos(angle);

    /* 各轮带符号行程，与 Chassis_Move 同一套逆解（W=0）。
       bx = 机体系横向分量(+x 横移)，by = 机体系前后分量(+y 前进)。
       全同速列对应前后，对角列对应横移，所以 by 走同速列、bx 走对角列。 */
    float s[4];
    s[0] = +bx + by;   /* 电机 addr 4 */
    s[1] = -bx + by;   /* 电机 addr 3 */
    s[2] = -bx + by;   /* 电机 addr 1 */
    s[3] = +bx + by;   /* 电机 addr 2 */

    chassis.line.last_x = target_x;
    chassis.line.last_y = target_y;
    chassis.setYaw = yaw;   /* 本次直线的目标航向，停车后由保持逻辑使用 */
    chassis.line.start_tick = osKernelGetTickCount();
    chassis.line.total_ms   = (uint32_t)time_ds * 100u;   /* 先按请求时长，下发后修正 */
    for (uint8_t i = 0; i < 4u; i++)
    {
        chassis.line.s_mm[i]     = s[i];
        chassis.line.start_mm[i] = s_wheel_mm[i];
    }
    s_line_settle = 0;
    chassis.line.seq++;
    chassis.line.done   = 0;
    chassis.line.active = 1;
    chassis.ctrlMode    = MOTOR_POS_MODE;
    taskEXIT_CRITICAL();

    /* CAN 下发放在临界区外（内部有 osDelay），返回预计实际时长；
       下发完成后再起计时，避免把 CAN 发送时间算进运动时间 */
    uint32_t dur_ms = Chassis_SendWheelPos(s, (float)time_ds * 0.1f, acc_pos);
    if (dur_ms == 0u)
    {
        dur_ms = (uint32_t)time_ds * 100u;
    }

    taskENTER_CRITICAL();
    chassis.line.start_tick = osKernelGetTickCount();
    chassis.line.total_ms   = dur_ms;
    taskEXIT_CRITICAL();

    return 1;
}

/* 直线移动到绝对坐标（指令坐标，世界系）。 */
uint8_t Chassis_Absolute_Pos_Ctrl_Line(int16_t target_x, int16_t target_y, uint8_t time_ds)
{
    return Chassis_LineMove((float)target_x, (float)target_y, time_ds, 0u);
}

/* 世界系相对移动：+y 前进、+x 横移，相对当前指令坐标；time_ds 单位 0.1s。
   返回 1=已受理，0=被拒绝（忙碌/时长 0/距离过小）。
   注意：位移相对的是"指令坐标"，不是当前车头方向。 */
uint8_t Chassis_Relative_Pos_Ctrl_Line(int16_t rel_x, int16_t rel_y, uint8_t time_ds)
{
    return Chassis_LineMove((float)rel_x, (float)rel_y, time_ds, 1u);
}







void Chassis_Run(void)
{

    /* 本周期只取一次航向：三次读之间若进了新帧，解算会用到两个不同时刻的 yaw */
    float yaw_now = Chassis_GetYawRad();

    /* 先消费上一周期到的新反馈，更新 Px/Py 和逻辑轮累计行程 */
    Chassis_UpdateOdometry(yaw_now);

    // chassis.Px =0- OFFSET * fast_sin(- yaw_now);
    // chassis.Py =0- OFFSET * (fast_cos(yaw_now) - 1);

    if (chassis.ctrlMode == VELOCITY_MODE)
    {
        /* 航向误差 = 目标 - 当前，直接作为 PID 的 set 传进去，保证误差不跨越 ±π */
        float err_yaw = Chassis_WrapPi(chassis.setYaw - yaw_now);

        float W;
        if (chassis.rotate_active)
        {
            /* 原地旋转：独立PID + speed 限幅，到位后自动结束 */
            PID_Calc(&chassis.PID_rotate, 0.0f, err_yaw);
            W = chassis.PID_rotate.out;
            if (W >  chassis.rotate_limitspeed) W =  chassis.rotate_limitspeed;
            if (W < -chassis.rotate_limitspeed) W = -chassis.rotate_limitspeed;

            if (fabsf(err_yaw) < CAR_ROTATE_TOL_DEG * DEG_TO_RAD)
            {
                chassis.rotate_active = 0;
                chassis.rotate_done = 1;
                chassis.PID_rotate.Iout = 0.0f;   /* 到位后清积分，防止下次残留 */
                W = 0.0f;                          /* 本周期直接停住，不再补一脚 */
            }
        }
        else
        {
            /* 给定 setVw 就按给定值；给 0 则保持锁存航向。
               给定 setVw 时不跑航向 PID，避免积分饱和；静止且误差在 ±容差 内输出 0。 */
            if (chassis.setVw != 0.0f)
            {
                chassis.PID_angle.Iout = 0.0f;     /* 自转时清积分，停车瞬间不跳 */
                W = chassis.setVw;
            }
            else if (chassis.setVx == 0.0f && chassis.setVy == 0.0f &&
                     fabsf(err_yaw) < CAR_ROTATE_TOL_DEG * DEG_TO_RAD)
            {
                chassis.PID_angle.Iout = 0.0f;     /* 死区内清积分，防止残留 */
                W = 0.0f;
            }
            else
            {
                PID_Calc(&chassis.PID_angle, 0.0f, err_yaw);
                W = chassis.PID_angle.out;
            }
        }
        Chassis_OutputVel(chassis.setVx, chassis.setVy, W, yaw_now);
    }
    else if (chassis.ctrlMode == MOTOR_POS_MODE)
    {
        /* 位移由驱动器位置环执行，到位后驱动器自动闭环保位。
           这里只负责在预计时长后标记完成，不再下发任何速度。
           注意：不能在运动中途再发位置指令做航向修正——那会把驱动器
           当前目标重设掉，导致主运动剩余的行程丢失、车只走一小段就停。 */
        uint32_t now = osKernelGetTickCount();

        if (chassis.line.active)
        {
            uint8_t done = 0;

            /* 位置到达：四轮实测行程都进入容差，连续 3 个周期确认 */
            if (Chassis_LineArrived())
            {
                if (++s_line_settle >= ODOM_SETTLE_N) { done = 1; }
            }
            else
            {
                s_line_settle = 0;
            }

            /* 里程计无效（还没 seed/掉线）或没到位时，按预计时长 + 余量兜底 */
            if ((!done) && ((now - chassis.line.start_tick) >= (chassis.line.total_ms + CHASSIS_POS_TIMEOUT_MS)))
            {
                done = 1;
            }

            if (done)
            {
                chassis.line.active = 0;
                chassis.line.done   = 1;
                chassis.ctrlMode    = STOP_MODE;
                chassis.yaw_hold_tick = now;   /* 稍后由 STOP_MODE 的保持逻辑修正航向 */
            }
        }

        if ((chassis.ctrlMode == MOTOR_POS_MODE) && chassis.rotate_active)
        {
            uint8_t done = 0;

            if (Chassis_RotateArrived())
            {
                if (++s_rot_settle >= ODOM_SETTLE_N) { done = 1; }
            }
            else
            {
                s_rot_settle = 0;
            }

            if ((!done) && ((now - chassis.rotate_start_tick) >= (chassis.rotate_total_ms + CHASSIS_POS_TIMEOUT_MS)))
            {
                done = 1;
            }

            if (done)
            {
                chassis.rotate_active = 0;
                chassis.rotate_done   = 1;
                chassis.ctrlMode      = STOP_MODE;
                chassis.yaw_hold_tick = now;
            }
        }
    }
    else if (chassis.ctrlMode == STOP_MODE)
    {
        /* 驱动器位置环已锁住目标位置；这里不下发任何速度。
           若航向被扰动或停下后漂出容差，用小角度原地旋转把它拉回。 */
        uint32_t now = osKernelGetTickCount();
        if ((now - chassis.yaw_hold_tick) >= CHASSIS_YAW_HOLD_PERIOD_MS)
        {
            chassis.yaw_hold_tick = now;
            float err = Chassis_WrapPi(chassis.setYaw - yaw_now);
            if (fabsf(err) > CAR_ROTATE_TOL_DEG * DEG_TO_RAD)   /* ±0.5° 死区 */
            {
                Chassis_RotateByRad(err, CHASSIS_YAW_FIX_RPM);
            }
        }
    }
    else if (chassis.ctrlMode == POSITION_MODE)
    {
        PID_Calc(&chassis.PID_angle, 0, Chassis_WrapPi(chassis.setYaw - yaw_now));
        /* 位置PID计算x轴速度, y轴速度 */
        PID_Calc(&chassis.PID_x, chassis.Px, chassis.setPx);
        PID_Calc(&chassis.PID_y, chassis.Py, chassis.setPy);

        /* 合速度大小 */
        float V = sqrtf(chassis.PID_x.out * chassis.PID_x.out + chassis.PID_y.out * chassis.PID_y.out);
        /* 合速度与Y轴夹角 */
        float angle = atan2f(chassis.PID_x.out, chassis.PID_y.out) + yaw_now;

        /* Vx */
        float Vx = V * fast_sin(angle);
        /* Vy */
        float Vy = V * fast_cos(angle);
        /* 底盘电机速度计算（与 Chassis_Move 同一套逆解，不要单独改符号） */
        chassis.Motors_Speed[0] = Chassis_ClampRpm(Vx + Vy - chassis.PID_angle.out);
        chassis.Motors_Speed[1] = Chassis_ClampRpm(Vx - Vy + chassis.PID_angle.out);
        chassis.Motors_Speed[2] = Chassis_ClampRpm(Vx - Vy - chassis.PID_angle.out);
        chassis.Motors_Speed[3] = Chassis_ClampRpm(Vx + Vy + chassis.PID_angle.out);
        Chassis_SetMotor_Vel(chassis.Motors_Speed);
    }

}

    /**
 * 设置底盘速度
 * 单位:mm/s
 * 单位:rad/s
 */
inline void Chassis_SetVel(float Vx, float Vy, float Vw)
{
    chassis.ctrlMode = VELOCITY_MODE;
    chassis.setVx = Vx;
    chassis.setVy = Vy;
    chassis.setVw = Vw;
    /* 手动速度指令抢占正在进行的原地旋转 / 直线移动 */
    chassis.rotate_active = 0;
    chassis.line.active   = 0;
    chassis.line.done     = 0;
    /* 把下发指令这一刻的航向锁存为目标航向；setVw 为 0 时 Chassis_Run 会用它做航向保持 */
    chassis.setYaw = Chassis_GetYawRad();
}

/**
 * 设置底盘位置
 * 单位:mm
 * 单位:rad
 */
inline void Chassis_SetPos(float Px, float Py, float Yaw)
{
    chassis.ctrlMode = POSITION_MODE;
    chassis.setPx = Px;
    chassis.setPy = Py;
    chassis.setYaw = Yaw;
}



