#include "Chassis_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "alg_fastmath.h"
#include "IMU_task.h"
#include "dvc_yis320.h"
#include <math.h>

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
uint8_t acc_pos=150;
int16_t  car_move_speed = 50;

chassis_t chassis = {0};   // 头文件里只有 extern 声明，定义放在这里

#define CHASSIS_WHEEL_DIAMETER_MM   80.0f      // 麦克纳姆轮直径
#define CHASSIS_PULSE_PER_REV       3200.0f    // Emm_V5：16 细分 × 200 步 = 3200 脉冲/圈
// 每毫米行程对应的脉冲数：3200 / (π × 80) ≈ 12.73
#define CHASSIS_PULSE_PER_MM  (CHASSIS_PULSE_PER_REV / (3.14159265f * CHASSIS_WHEEL_DIAMETER_MM))

#define CHASSIS_POS_VEL_RPM  200        // 位置模式的行进速度上限(RPM)，四轮按脉冲比例缩放
#define DEG_TO_RAD  (PI / 180.0f)

void Chassis_Init(void)
{
    AttachInterrupt_FDCAN(&hfdcan1, ZDT_CAN_Callback);
    // Emm_V5_En_Control(2, true, true);                 // 使能电机
    osDelay(500);


//   /* 底盘x坐标pid初始化 */
//     PID_Init(&chassis.PID_x, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
//     /* 底盘y坐标pid初始化 */
//     PID_Init(&chassis.PID_y, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
    /* 底盘角度pid初始化 */
    PID_Init(&chassis.PID_angle, 2.0, 0.01f, 3.0f, 3.0f, 0.1f);



}

void Chassis_Task(void *argument)
{
    (void)argument;
    Chassis_Init();
    // Chassis_SetVel(0, 100, 0);

    Chassis_MovePos(0.0f, 0.0f, 90.0f);   /* 上电前进 100mm */
    for (;;)
    {
        Chassis_Run();
        osDelay(10);               // 速度模式调试时改回 Chassis_Move(...)
    }
}


// void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W)
// {
//     int16_t speed[5];

//     speed[1] = +Vy +Vx -W;
// 	speed[2] = -Vy + Vx +W;
// 	speed[3] = -Vy + Vx -W;
// 	speed[4] = +Vy + Vx +W;

//     Chassis_SetMotor_Vel(&speed);
// }





/* 单轮位置下发：pulses 带符号，正向/反向的方向码按电机安装方向给。
   raF=2 表示相对当前实时位置，连续两次下发互不累加。 */
static void Chassis_SetOneMotorPos(uint8_t addr, uint8_t dir_pos, uint8_t dir_neg,
                                   int32_t pulses, uint16_t vel, uint8_t acc)
{
    uint8_t dir = (pulses >= 0) ? dir_pos : dir_neg;
    uint32_t p = (uint32_t)((pulses >= 0) ? pulses : -pulses);

    Emm_V5_Pos_Control(addr, dir, vel, acc, p, 2, true);
}

/* pulses[0..3]：左上(ID4)、右上(ID3)、左下(ID1)、右下(ID2)，单位脉冲，带符号。
   四轮 RPM 按脉冲比例缩放，同步启动后才能同时到达，路径不会走弯。 */
void Chassis_SetMotor_Pos(const int32_t pulses[4], uint16_t vel_rpm, uint8_t acc)
{
    int32_t mag[4];
    int32_t maxp = 0;
    uint16_t vel[4];
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        mag[i] = (pulses[i] >= 0) ? pulses[i] : -pulses[i];
        if (mag[i] > maxp) maxp = mag[i];
    }

    for (i = 0; i < 4; i++)
    {
        if (maxp == 0 || mag[i] == 0)
        {
            vel[i] = 0;
        }
        else
        {
            vel[i] = (uint16_t)((uint32_t)vel_rpm * (uint32_t)mag[i] / (uint32_t)maxp);
            if (vel[i] == 0) vel[i] = 1;    // 不能给 0 RPM，驱动器不会动
        }
    }

    if (mag[0]) { Chassis_SetOneMotorPos(4, 0, 1, pulses[0], vel[0], acc); }
    osDelay(1);
    if (mag[1]) { Chassis_SetOneMotorPos(3, 0, 1, pulses[1], vel[1], acc); }
    osDelay(1);
    if (mag[2]) { Chassis_SetOneMotorPos(1, 1, 0, pulses[2], vel[2], acc); }
    osDelay(1);
    if (mag[3]) { Chassis_SetOneMotorPos(2, 1, 0, pulses[3], vel[3], acc); }
    osDelay(1);

    Emm_V5_Synchronous_motion(0); //广播触发
}

/* 相对当前位姿移动：dx/dy 单位 mm（车体系），dtheta 单位度（逆时针为正）。
   走驱动器内置位置模式，开环、不需要底盘里程计。 */
void Chassis_MovePos(float dx_mm, float dy_mm, float dtheta_deg)
{
    float dth = dtheta_deg * DEG_TO_RAD;
    float ds[4];
    int32_t pulses[4];
    uint8_t i;

    /* 与 Chassis_Run 的速度逆解同一套符号，把速度积分成位移 */
    ds[0] = dx_mm + dy_mm - dth * LEGNTH_WIDTH;   /* 左上 ID4 */
    ds[1] = dx_mm - dy_mm + dth * LEGNTH_WIDTH;   /* 右上 ID3 */
    ds[2] = dx_mm - dy_mm - dth * LEGNTH_WIDTH;   /* 左下 ID1 */
    ds[3] = dx_mm + dy_mm + dth * LEGNTH_WIDTH;   /* 右下 ID2 */

    for (i = 0; i < 4; i++)
    {
        pulses[i] = (int32_t)lroundf(ds[i] * CHASSIS_PULSE_PER_MM);
    }

    chassis.ctrlMode = MOTOR_POS_MODE;
    Chassis_SetMotor_Pos(pulses, CHASSIS_POS_VEL_RPM, acc_pos);
}

/* 单轮速度下发：v 为 RPM（带符号），按电机安装方向取 dir，幅值钳位到 0~5000 */
static void Chassis_SetOneMotorVel(uint8_t addr, uint8_t dir_pos, uint8_t dir_neg, float v)
{
    uint8_t dir = (v >= 0.0f) ? dir_pos : dir_neg;
    uint32_t rpm = (uint32_t)(fabsf(v) + 0.5f);

    if (rpm > 5000u) rpm = 5000u;

    Emm_V5_Vel_Control(addr, dir, (uint16_t)rpm, acc_vel, false);
}

/* speed[0..3]：左上(ID4)、右上(ID3)、左下(ID1)、右下(ID2)，单位 RPM */
void Chassis_SetMotor_Vel(const float speed[4])
{
    Chassis_SetOneMotorVel(4, 0, 1, speed[0]);
	osDelay(1);

	Chassis_SetOneMotorVel(3, 0, 1, speed[1]);
	osDelay(1);

	Chassis_SetOneMotorVel(1, 1, 0, speed[2]);
	osDelay(1);

	Chassis_SetOneMotorVel(2, 1, 0, speed[3]);
    osDelay(1);
}


/* 角度归一化到 (-PI, PI]，避免 yaw 在 ±180° 处跳变把误差放大成 2π */
static float Chassis_WrapPi(float a)
{
    while (a > PI)  { a -= 2.0f * PI; }
    while (a < -PI) { a += 2.0f * PI; }
    return a;
}

/* 底盘航向来源：YIS320 的 yaw 是度、范围 [-180,180)，转成 rad 与 setYaw 同量纲。
   不要再混用 imuAngle[INS_YAW_ADDRESS_OFFSET]：两路姿态零点和漂移都不一样，
   混用会让航向 PID 追着一个不存在的误差。 */
static inline float Chassis_GetYawRad(void)
{
    return IMU_getYaw() * DEG_TO_RAD;
}

void Chassis_Run(void)
{
    /* 位置指令已经下发，电机在位置模式里自己闭环；
       再用速度指令覆盖会让电机退出位置保持，位移就废了。 */
    if (chassis.ctrlMode == MOTOR_POS_MODE) return;

    //  chassis.Px = Action.pos_x - OFFSET * fast_sin(-RAD(Action.yaw));
    // chassis.Py = Action.pos_y - OFFSET * (fast_cos(RAD(Action.yaw)) - 1);

    /* 本周期只取一次航向：三次读之间若进了新帧，解算会用到两个不同时刻的 yaw */
    float yaw_now = Chassis_GetYawRad();

    /* 航向误差 = 目标 - 当前，直接作为 PID 的 set 传进去，保证误差不跨越 ±π */
    PID_Calc(&chassis.PID_angle, 0.0f,
             Chassis_WrapPi(chassis.setYaw - yaw_now));

    if (chassis.ctrlMode == VELOCITY_MODE)
    {
        /* 自转角速度：给定 setVw 就按给定值，给 0 则用航向 PID 保持锁存的目标航向 */
        float W = (chassis.setVw == 0.0f) ? chassis.PID_angle.out : chassis.setVw;

        /* 合速度大小 */
        float V = sqrtf(chassis.setVx * chassis.setVx + chassis.setVy * chassis.setVy);
        /* 合速度与Y轴夹角 */
        float angle = atan2f(chassis.setVx, chassis.setVy) + yaw_now;
        /* Vx */
        float Vx = V * fast_sin(angle);
        /* Vy */
        float Vy = V * fast_cos(angle);
        /* 底盘电机速度计算（与 Chassis_Move 同一套逆解，不要单独改符号） */
        chassis.Motors_Speed[0] = (Vx + Vy - W * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[1] = (Vx - Vy + W * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[2] = (Vx - Vy - W * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[3] = (Vx + Vy + W * LEGNTH_WIDTH) * COEFFICIENT;

        Chassis_SetMotor_Vel(chassis.Motors_Speed);
    }
    else if (chassis.ctrlMode == POSITION_MODE)
    {

        // /* 位置PID计算x轴速度, y轴速度 */
        // PID_Calc(&chassis.PID_x, chassis.Px, chassis.setPx);
        // PID_Calc(&chassis.PID_y, chassis.Py, chassis.setPy);
        /* PID_angle 已在函数开头算过，这里不要重复调用（会重复积分/重复差分） */

        /* 合速度大小 */
        float V = sqrtf(chassis.PID_x.out * chassis.PID_x.out + chassis.PID_y.out * chassis.PID_y.out);
        /* 合速度与Y轴夹角 */
        float angle = atan2f(chassis.PID_x.out, chassis.PID_y.out) + yaw_now;

        /* Vx */
        float Vx = V * fast_sin(angle);
        /* Vy */
        float Vy = V * fast_cos(angle);
        /* 底盘电机速度计算（与 Chassis_Move 同一套逆解，不要单独改符号） */
        chassis.Motors_Speed[0] = (Vx + Vy - chassis.PID_angle.out * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[1] = (Vx - Vy + chassis.PID_angle.out * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[2] = (Vx - Vy - chassis.PID_angle.out * LEGNTH_WIDTH) * COEFFICIENT;
        chassis.Motors_Speed[3] = (Vx + Vy + chassis.PID_angle.out * LEGNTH_WIDTH) * COEFFICIENT;
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



