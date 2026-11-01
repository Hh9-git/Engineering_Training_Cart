#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "alg_pid.h"

#define RADIUS 152.0f            // 轮子直径
#define LENGTH 130.0f            // 底盘半长
#define WIDTH 188.0f             // 底盘半宽
#define LEGNTH_WIDTH 318.0f      // 半长半宽之和
#define RR 228.57f               // 轮子到中心的距离
#define DECRATIO 19              // 电机减速比
#define OFFSET 116.0f            // 码盘偏移
#define COEFFICIENT 2.387324146f // 单位系数

typedef enum
{
    VELOCITY_MODE,
    POSITION_MODE,
    MOTOR_POS_MODE,     // 已下发驱动器位置指令，Chassis_Run 不再输出速度
} chassis_ctrl_e;


typedef struct
{
    /********************** 电机 ************************/
    // DJ_Motor_t ChassisMotors[8];

    float Motors_Speed[8]; // 电机速度

    float Px;
    float Py;

    /********************** 底盘参数 ************************/
    chassis_ctrl_e ctrlMode; // 控制模式

    float setVx; // 底盘设置x速度
    float setVy; // 底盘设置y速度
    float setVw; // 底盘设置角速度

    float setPx;  // 底盘设置x坐标, 单位mm
    float setPy;  // 底盘设置y坐标, 单位mm
    float setYaw; // 偏航角, 绕z轴, 单位rad

    // TDPID_t PID_Angle; // 角度pid
    // TDPID_t PID_X;     // x坐标pid
    // TDPID_t PID_Y;     // y坐标pid

    PID_t PID_angle; // 角度pid
    PID_t PID_x;     // x坐标pid
    PID_t PID_y;     // y坐标pid
} chassis_t;



extern chassis_t chassis;


void Chassis_Init(void);
void Chassis_Task(void *argument);

void Chassis_Run(void);
void Chassis_SetVel(float Vx, float Vy, float Vw);
void Chassis_SetPos(float Px, float Py, float Yaw);
void Chassis_MovePos(float dx_mm, float dy_mm, float dtheta_deg);

void Chassis_SetMotor_Vel(const float speed[4]);

void Chassis_SetMotor_Pos(const int32_t pulses[4], uint16_t vel_rpm, uint8_t acc);
static float Chassis_WrapPi(float a);

#endif // CHASSIS_TASK_H