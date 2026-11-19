#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "main.h"
#include "alg_pid.h"

/* RC底盘几何参数 
#define RADIUS80  152.0f            // 轮子直径
#define LENGTH 130.0f            // 底盘半长
#define WIDTH 188.0f             // 底盘半宽
#define LEGNTH_WIDTH 315.0f      // 半长半宽之和
#define RR 228.57f               // 轮子到中心的距离
#define DECRATIO 1             // 电机减速比
#define OFFSET 116.0f            // 码盘偏移
#define COEFFICIENT 2.387324146f // 单位系数
*/

#define CHASSIS_WHEEL_RADIUS       40.0f
#define CHASSIS_POS_COUNT_PER_REV  65536.0f

#define CHASSIS_PI                 3.1415926f

#define CHASSIS_POS_TO_MM \
    (2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS / CHASSIS_POS_COUNT_PER_REV)
typedef enum
{
    VELOCITY_MODE,
    POSITION_MODE,
    SPIN_DRIFT_MODE,
    ROTATE_MODE
} Chassis_CtrlMode_e;
typedef struct
{
    /********************** 位置 ************************/
    float Px;                    // 当前X坐标，mm
    float Py;                    // 当前Y坐标，mm

    /********************** 控制模式 ************************/
    Chassis_CtrlMode_e ctrlMode;

    /********************** 速度控制 ************************/
    float setVx;                 // 目标X速度，mm/s
    float setVy;                 // 目标Y速度，mm/s
    float setVw;                 // 目标角速度，具体单位待统一

    float nowVx;                 // 当前X方向速度，mm/s
    float nowVy;                 // 当前Y方向速度，mm/s

    /********************** 位置控制 ************************/
    float setPx;                 // 目标X坐标，mm
    float setPy;                 // 目标Y坐标，mm
    float setYaw;                // 目标偏航角，°

    /********************** PID ************************/
    PID_t PID_angle;
    PID_t PID_x;
    PID_t PID_y;

    /********************** 加减速控制 ************************/
    uint16_t accel_count;
    uint16_t accel_K;

} chassis_t;



extern chassis_t chassis;

void Chassis_Init(void);
void Chassis_Task(void *argument);
void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W);

void Chassis_clear(void);
void Chassis_Motor_SetVel(int16_t speed[4]);
void Chassis_SetVel(float Vx, float Vy);
void Chassis_SetPos(float Px, float Py, float Yaw);
void Chassis_SetAngle(float target);
void Chassis_Update(void);


void Chassis_Drift(float Vx, float Vy, float Yaw);
void Chassis_SpinDrift(float Vx, float Vy, float W);

void Chassis_Odometry_Update(void);
void Chassis_ResetPos(void);

void Chassis_Rotate(float target);

#endif // CHASSIS_TASK_H