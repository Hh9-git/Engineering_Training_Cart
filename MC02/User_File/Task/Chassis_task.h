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




#define CHASSIS_WHEEL_DIAMETER_MM   80.0f      // 麦克纳姆轮直径
#define CHASSIS_PULSE_PER_REV       3200.0f    // Emm_V5：16 细分 × 200 步 = 3200 脉冲/圈
// 每毫米行程对应的脉冲数：3200 / (π × 80) ≈ 12.73
#define CHASSIS_PULSE_PER_MM  (CHASSIS_PULSE_PER_REV / (3.14159265f * CHASSIS_WHEEL_DIAMETER_MM))

#define CHASSIS_POS_VEL_RPM  200        // 位置模式的行进速度上限(RPM)，四轮按脉冲比例缩放
#define DEG_TO_RAD  (PI / 180.0f)

/* 直线移动的最小位移，太小不下发 */
#define CHASSIS_LINE_MIN_DIST  1.0f
/* 单次直线最大位移(mm)，超过直接拒绝，防止坐标回绕/误传导致冲出去 */
#define CHASSIS_LINE_MAX_DIST_MM  10000.0f

/* 麦轮旋转力臂(mm)：轮子行程 = 旋转角(rad) × 该值。
   实测标定：用 LEGNTH_WIDTH=315 时 90° 指令实际转了 135°，故 315 × 90/135 = 210。 */
#define CHASSIS_ROTATE_ARM_MM  210.0f

/* 到点后航向修正的旋转速度(RPM) */
#define CHASSIS_YAW_FIX_RPM  40u
/* 静止时航向保持的检查周期 */
#define CHASSIS_YAW_HOLD_PERIOD_MS  300u

#define CAR_ROTATE_TOL_DEG 0.5f  // 原地旋转到位/保持容差, 单位度(±)

typedef enum
{
    VELOCITY_MODE,
    POSITION_MODE,
    MOTOR_POS_MODE,     // 已下发驱动器位置指令，Chassis_Run 不再输出速度
    STOP_MODE,          // 到点停车：驱动器已锁位，Chassis_Run 不再下发速度
} chassis_ctrl_e;


/* 直线移动状态。位移由驱动器位置环执行，last_x/last_y 只是指令位置积分（本工程无里程计）。 */
typedef struct
{
    uint32_t start_tick;    // 下发时刻 osKernelGetTickCount()，ms
    uint32_t total_ms;      // 预计运动时长，ms
    uint32_t seq;           // 每次成功下发 +1
    float    last_x;        // 指令位置，mm
    float    last_y;
    float    s_mm[4];       // 本次直线每轮目标行程，逻辑轮顺序 {addr4, addr3, addr1, addr2}
    float    start_mm[4];   // 下发时刻逻辑轮累计行程快照，mm
    uint8_t  active;        // 1: 直线移动进行中
    uint8_t  done;          // 1: 已完成
} CarCtrl_Line;


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
    uint32_t rotate_start_tick; // 位置式旋转起始 tick
    uint32_t rotate_total_ms;   // 位置式旋转预计时长
    uint32_t yaw_hold_tick;     // 上次航向保持检查的 tick
    float    rotate_s_mm[4];    // 本次旋转每轮目标行程
    float    rotate_start_mm[4];// 旋转下发时刻逻辑轮累计行程快照

    /********************** 直线移动 ************************/
    CarCtrl_Line line;
} chassis_t;



extern chassis_t chassis;



typedef struct
{
    uint16_t    target_time_count_X;
    uint16_t    target_time_count_Y;
    float       Current_Speed_X;
    float       Current_Speed_Y;
    float       Max_Speed_X;
    float       Max_Speed_Y;
}CarCtrl_Curve;


void Chassis_Init(void);
void Chassis_Task(void *argument);

void Chassis_Run(void);
void Chassis_SetVel(float Vx, float Vy, float Vw);
void Chassis_SetPos(float Px, float Py, float Yaw);
void Chassis_MovePos(float dx_mm, float dy_mm, float dtheta_deg);

void Chassis_SetMotor_Vel(int16_t speed[4]);
void Chassis_SetMotor_Pos(int16_t speed[4],uint32_t clk);





void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W);
void Chassis_Rotate(int16_t angle, uint8_t speed);
uint8_t Chassis_Absolute_Pos_Ctrl_Line(int16_t target_x, int16_t target_y, uint8_t time_ds);
uint8_t Chassis_Relative_Pos_Ctrl_Line(int16_t rel_x, int16_t rel_y, uint8_t time_ds);

#endif // CHASSIS_TASK_H