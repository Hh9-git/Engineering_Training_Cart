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

}

void Chassis_Task(void *argument)
{
    (void)argument;
    Chassis_Init();
    // Car_Absolute_Pos_Ctrl_Line(0, 2000, 30);   /* 上电测试：前进 2m / 3s，验证完请删除 */
    Car_Rotate(180, 100);   /* 上电测试：顺时针旋转 90° / 50RPM，验证完请删除 */
    for (;;)
    {
        Chassis_Run();
        osDelay(10);               // 速度模式调试时改回 Chassis_Move(...)
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
static void Car_RotateByRad(float dtheta, uint8_t speed)
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
void Car_Rotate(int16_t angle, uint8_t speed)
{
    if (angle < -180 || angle > 180) return;
    if (chassis.rotate_active || chassis.line.active) return;  /* 对应旧版 flag.car_in_task == 0 封锁 */
    Car_RotateByRad((float)angle * DEG_TO_RAD - Chassis_GetYawRad(), speed);
}




/* 直线移动到绝对坐标（指令坐标）。位移由 Emm_V5 位置环执行：
   四轮按逆解分配脉冲数、按比例分配速度、同时到达 → 直行且航向不变，
   到位后驱动器闭环保位，不会漂。
   target_x/target_y: mm，+y 前进、+x 横移，与上电原点同一坐标系
   time_ds: 期望总时长，单位 0.1s（0.1~25.5s），决定运动速度
   返回 1=已受理，0=被拒绝。只能在任务上下文调用，不要在中断里调。 */
uint8_t Car_Absolute_Pos_Ctrl_Line(uint16_t target_x, uint16_t target_y, uint8_t time_ds)
{
    if (time_ds == 0) return 0;

    float dx = (float)target_x - chassis.line.last_x;
    float dy = (float)target_y - chassis.line.last_y;
    float distance = sqrtf(dx * dx + dy * dy);
    if (distance < CHASSIS_LINE_MIN_DIST) return 0;

    /* 检查并置忙必须原子，避免两个任务同时通过守卫 */
    taskENTER_CRITICAL();
    if (chassis.line.active || chassis.rotate_active)
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

    chassis.line.last_x = (float)target_x;
    chassis.line.last_y = (float)target_y;
    chassis.setYaw = yaw;   /* 本次直线的目标航向，停车后由保持逻辑使用 */
    chassis.line.start_tick = osKernelGetTickCount();
    chassis.line.total_ms   = (uint32_t)time_ds * 100u;   /* 先按请求时长，下发后修正 */
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




void Chassis_Run(void)
{

    /* 本周期只取一次航向：三次读之间若进了新帧，解算会用到两个不同时刻的 yaw */
    float yaw_now = Chassis_GetYawRad();

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
               静止且误差在 ±容差 内时输出 0：到位后停住，不持续微调/蠕动；
               被推动超出容差后航向 PID 会重新把车拉回来。 */
            if (chassis.setVw == 0.0f &&
                chassis.setVx == 0.0f && chassis.setVy == 0.0f &&
                fabsf(err_yaw) < CAR_ROTATE_TOL_DEG * DEG_TO_RAD)
            {
                chassis.PID_angle.Iout = 0.0f;     /* 死区内清积分，防止残留 */
                W = 0.0f;
            }
            else
            {
                PID_Calc(&chassis.PID_angle, 0.0f, err_yaw);
                W = (chassis.setVw == 0.0f) ? chassis.PID_angle.out : chassis.setVw;
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

        if (chassis.line.active &&
            (now - chassis.line.start_tick) >= chassis.line.total_ms)
        {
            chassis.line.active = 0;
            chassis.line.done   = 1;
            chassis.ctrlMode    = STOP_MODE;
            chassis.yaw_hold_tick = now;   /* 稍后由 STOP_MODE 的保持逻辑修正航向 */
        }
        if (chassis.rotate_active &&
            (now - chassis.rotate_start_tick) >= chassis.rotate_total_ms)
        {
            chassis.rotate_active = 0;
            chassis.rotate_done   = 1;
            chassis.ctrlMode      = STOP_MODE;
            chassis.yaw_hold_tick = now;
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
                Car_RotateByRad(err, CHASSIS_YAW_FIX_RPM);
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



