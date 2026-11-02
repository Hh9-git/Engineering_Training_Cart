#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "alg_pid.h"

#define RADIUS 152.0f            // 轮子直径
#define LENGTH 130.0f            // 底盘半长
#define WIDTH 188.0f             // 底盘半宽
#define LEGNTH_WIDTH 315.0f      // 半长半宽之和
#define RR 228.57f               // 轮子到中心的距离
#define DECRATIO 19              // 电机减速比
#define OFFSET 116.0f            // 码盘偏移
#define COEFFICIENT 2.387324146f // 单位系数

#define CAR_ROTATE_TOL_DEG 1.0f  // 原地旋转到位容差, 单位度

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

    int16_t Motors_Speed[8]; // 电机速度, 单位RPM(带符号), 驱动器范围为0~5000

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

    /********************** 原地旋转 ************************/
    PID_t PID_rotate;        // 旋转专用航向pid, 与PID_angle分开调参
    float rotate_limitspeed; // 旋转时W的限幅, 单位RPM
    uint8_t rotate_active;   // 1: 旋转进行中(对应旧版 car_in_task != 0)
    uint8_t rotate_done;     // 1: 已到达目标航向
} chassis_t;



extern chassis_t chassis;


void Chassis_Init(void);
void Chassis_Task(void *argument);

void Chassis_Run(void);
void Chassis_SetVel(float Vx, float Vy, float Vw);
void Chassis_SetPos(float Px, float Py, float Yaw);
void Chassis_MovePos(float dx_mm, float dy_mm, float dtheta_deg);

void Chassis_SetMotor_Vel(int16_t speed[4]);
void Chassis_SetMotor_Pos(int16_t speed[4],uint32_t clk);





void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W);
void Car_Rotate(int16_t angle, uint8_t speed);

#endif // CHASSIS_TASK_H