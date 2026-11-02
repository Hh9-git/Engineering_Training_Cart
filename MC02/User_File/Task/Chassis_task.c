#include "Chassis_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "alg_fastmath.h"
#include "IMU_task.h"
#include "dvc_yis320.h"
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
uint8_t acc_pos=0;
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


  /* 底盘x坐标pid初始化 */
    PID_Init(&chassis.PID_x, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
    /* 底盘y坐标pid初始化 */
    PID_Init(&chassis.PID_y, 4.0, 0.01f, 0.9f, 1500.0f, 3.0f);
    /* 底盘角度pid初始化 */
    PID_Init(&chassis.PID_angle, 2.0, 0.01f, 3.0f, 3.0f, 0.1f);
    /* 旋转航向pid初始化：err 单位rad，输出单位近似RPM。以下为起步值，需上车标定 */
    PID_Init(&chassis.PID_rotate, 100.0f, 0.5f, 5.0f, 500.0f, 100.0f);

}

void Chassis_Task(void *argument)
{
    (void)argument;
    Chassis_Init();
    Car_Rotate(90, 100);

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
	if(speed[1]>=0){Emm_V5_Pos_Control(3, 0, speed[1], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(3, 1, -speed[1], acc_pos, clk, 0, true);}
	osDelay(1);

    //正向
	if(speed[2]>=0){Emm_V5_Pos_Control(1, 1, speed[2], acc_pos, clk, 0, true);}else{Emm_V5_Pos_Control(1, 0, -speed[2], acc_pos, clk, 0, true);}
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










/* 原地旋转到绝对目标航向。
   angle: 目标航向(度), 逆时针为正, 范围[-180,180], 与 IMU_getYaw() 同坐标系(上电为零点)
   speed: 旋转时 W(角速度分量)的绝对值上限, 单位RPM
   只下发目标并置忙，实际旋转由 Chassis_Run 的 PID_rotate 闭环完成；非阻塞。 */
void Car_Rotate(int16_t angle, uint8_t speed)
{
    if (angle < -180 || angle > 180) return;
    if (chassis.rotate_active) return;          /* 对应旧版 flag.car_in_task == 0 封锁 */

    chassis.ctrlMode = VELOCITY_MODE;
    chassis.setVx = 0.0f;
    chassis.setVy = 0.0f;
    chassis.setVw = 0.0f;                       /* setVw==0 时 W 由航向PID给出 */
    chassis.setYaw = (float)angle * DEG_TO_RAD; /* 绝对目标航向(rad) */

    /* PID_Init 不清状态，重新进入旋转前手动清一次，避免沿用上次累计 */
    chassis.PID_rotate.Iout = 0.0f;
    chassis.PID_rotate.err[0] = 0.0f;
    chassis.PID_rotate.err[1] = 0.0f;

    chassis.rotate_limitspeed = (float)speed;
    chassis.rotate_done = 0;
    chassis.rotate_active = 1;
}

void Chassis_Run(void)
{

    /* 本周期只取一次航向：三次读之间若进了新帧，解算会用到两个不同时刻的 yaw */
    float yaw_now = Chassis_GetYawRad();

    chassis.Px =0- OFFSET * fast_sin(- yaw_now);
    chassis.Py =0- OFFSET * (fast_cos(yaw_now) - 1);

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
            }
        }
        else
        {
            /* 自转角速度：给定 setVw 就按给定值，给 0 则用航向 PID 保持锁存的目标航向 */
            PID_Calc(&chassis.PID_angle, 0.0f, err_yaw);
            W = (chassis.setVw == 0.0f) ? chassis.PID_angle.out : chassis.setVw;
        }
        /* 合速度大小 */
        float V = sqrtf(chassis.setVx * chassis.setVx + chassis.setVy * chassis.setVy);
        /* 合速度与Y轴夹角 */
        float angle = atan2f(chassis.setVx, chassis.setVy) + yaw_now;
        /* Vx */
        float Vx = V * fast_sin(angle);
        /* Vy */
        float Vy = V * fast_cos(angle);
        /* 底盘电机速度计算（与 Chassis_Move 同一套逆解，不要单独改符号） */
        chassis.Motors_Speed[0] = Chassis_ClampRpm(Vx + Vy - W);
        chassis.Motors_Speed[1] = Chassis_ClampRpm(Vx - Vy + W);
        chassis.Motors_Speed[2] = Chassis_ClampRpm(Vx - Vy - W);
        chassis.Motors_Speed[3] = Chassis_ClampRpm(Vx + Vy + W);

        Chassis_SetMotor_Vel(chassis.Motors_Speed);
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
    /* 手动速度指令抢占正在进行的原地旋转 */
    chassis.rotate_active = 0;
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



