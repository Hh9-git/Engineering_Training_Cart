#include "Tower_control.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"
#include "Tower_action.h"



static Tower_Motor_Target_t lift_target = {0};
static Tower_Motor_Target_t telescope_target = {0};

static int16_t gimbal_target_angle = 0;
static uint8_t gimbal_update_flag = 0;

static int16_t claw_target_angle = 0;
static uint8_t claw_update_flag = 0;


static bool Tower_Update_Lift(void);
static bool Tower_Update_Telescope(void);

#define TOWER_CMD_MAX_RETRY 100

void Tower_Control_Init(void)
{
    // 初始化舵机
    Servo_Init();
// Tower_Control(30, 90, 1000, 100, 1000);
 
/* 复位姿态 */
Tower_Target_Update(0, 50, 15);
// Tower_SetClaw_Angle(45);  // 打开

}




// 夹爪舵机目标角度设置：只保存目标
void Tower_SetClaw_Angle(int16_t angle)
{
    claw_target_angle = angle;
    claw_update_flag = 1;
}

// 云台绝对角度控制，angle 表示云台相对于舵机设定中位（0°）的目标角度
void Tower_SetGimbal_Angle(int16_t angle)
{
    gimbal_target_angle = angle;
    gimbal_update_flag = 1;
}
// 升降机构绝对位置控制：只保存目标参数
void Tower_SetLift_Pos(int16_t target_pos,
                       uint16_t speed,
                       uint8_t acc)
{
    // 限制运动范围，以齿条为界
    if (target_pos > 130)
        target_pos = 130;

    if (target_pos < 20)
        target_pos = 20;

    // 保存目标参数
    lift_target.target_pos = target_pos;
    lift_target.speed = speed;
    lift_target.acc = acc;
    lift_target.ack_base = Emm_Feedback[5].pos_ack_cnt;
    lift_target.retry = 0;
    lift_target.update_flag = 1;
}

// 伸缩机构绝对位置控制：只保存目标参数
void Tower_SetTelescope_Pos(int16_t target_pos,
                            uint16_t speed,
                            uint8_t acc)
{
    // 限制运动范围，以夹爪舵机后端为界
    if (target_pos > 130)
        target_pos = 130;

    if (target_pos < 5)
        target_pos = 5;

    // 保存目标参数
    telescope_target.target_pos = target_pos;
    telescope_target.speed = speed;
    telescope_target.acc = acc;
    telescope_target.ack_base = Emm_Feedback[6].pos_ack_cnt;
    telescope_target.retry = 0;
    telescope_target.update_flag = 1;
}

void Tower_Control(uint32_t yaw,
                   uint32_t z,
                   uint16_t z_speed,
                   uint32_t y,
                   uint16_t y_speed)
{
    // 云台舵机：设置目标角度
    Tower_SetGimbal_Angle((int16_t)yaw);

    // 升降机构 ID5：设置目标位置
    Tower_SetLift_Pos((int16_t)z, z_speed, 0);

    // 伸缩机构 ID6：设置目标位置
    Tower_SetTelescope_Pos((int16_t)y, y_speed, 0);
}

// Tower 电机周期运行函数
void Tower_Motor_Run(void)
{

     // 云台舵机控制
    if (gimbal_update_flag)
    {
        Set_Gimbal_Angle(gimbal_target_angle);
        gimbal_update_flag = 0;
    }

    // 夹爪舵机控制
    if (claw_update_flag)
    {
        Set_Claw_Angle(claw_target_angle);
        claw_update_flag = 0;
    }


    static uint8_t turn = 0u;
    // 两个电机交替发，一个没应答也不会把另一个饿死
    turn ^= 1u;

    if (turn != 0u)
    {
        if (!Tower_Update_Lift())
        {
            (void)Tower_Update_Telescope();
        }
    }
    else
    {
        if (!Tower_Update_Telescope())
        {
            (void)Tower_Update_Lift();
        }
    }
}


// 发送一次升降电机命令。
// 返回 true 表示这一轮真的发了；false 表示没发（没有待发目标，或本次命令已确认完成）。
static bool Tower_Update_Lift(void)
{
    if (!lift_target.update_flag)
        return false;

    // 已经发过至少一次，且电机回了 02（或重发次数用完）→ 本次命令结束
    if ((lift_target.retry >= TOWER_CMD_MAX_RETRY) ||
        ((lift_target.retry > 0u) &&
         (Emm_Feedback[5].pos_ack_cnt != lift_target.ack_base)))
    {
        lift_target.update_flag = 0;
        return false;
    }

    // 每个新目标的第一发之前先使能：上电后 X42S 若处于未使能状态，命令收到也不动
    if (lift_target.retry == 0u)
    {
        Emm_V5_En_Control(5, true, false);
    }
    lift_target.retry++;

    // 根据目标位置计算脉冲数
    uint32_t pulses = (uint32_t)((float)lift_target.target_pos
                                 * LIFT_PULSE_PER_MM);

    // 发送升降电机位置指令（绝对模式，重发幂等）
    (void)Emm_V5_Pos_Control(5,
                             1,                     // 1：上行
                             lift_target.speed,
                             lift_target.acc,
                             pulses,
                             1,                     // 绝对位置模式
                             false);
    return true;
}

// 发送一次伸缩电机命令。
// 返回 true 表示这一轮真的发了；false 表示没发（没有待发目标，或本次命令已确认完成）。
static bool Tower_Update_Telescope(void)
{
    if (!telescope_target.update_flag)
        return false;

    if ((telescope_target.retry >= TOWER_CMD_MAX_RETRY) ||
        ((telescope_target.retry > 0u) &&
         (Emm_Feedback[6].pos_ack_cnt != telescope_target.ack_base)))
    {
        telescope_target.update_flag = 0;
        return false;
    }

    if (telescope_target.retry == 0u)
    {
        Emm_V5_En_Control(6, true, false);
    }
    telescope_target.retry++;

    // 根据目标位置计算脉冲数
    uint32_t pulses = (uint32_t)((float)telescope_target.target_pos
                                 * TELESCOPE_PULSE_PER_MM);

    // 发送伸缩电机位置指令（绝对模式，重发幂等）
    (void)Emm_V5_Pos_Control(6,
                             0,                     // 0：伸出
                             telescope_target.speed,
                             telescope_target.acc,
                             pulses,
                             1,                     // 绝对位置模式
                             false);
    return true;
}

