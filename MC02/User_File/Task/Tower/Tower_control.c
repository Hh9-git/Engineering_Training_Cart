#include "Tower_control.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"


#define LIFT_PULSE_PER_MM 83.33f

void Tower_Control_Init(void)
{
    Servo_Init();
    
    Tower_SetLift_Pos(0, 5000, 0);
}

// 云台绝对角度控制，angle 表示云台相对于舵机设定中位（0°）的目标角度
void Tower_SetGimbal_Angle(int16_t angle)
{
    Set_Gimbal_Angle(angle);
}

// 升降机构绝对位置控制
void Tower_SetLift_Pos(int16_t target_pos,
                       uint16_t speed,
                       uint8_t acc)
{
    // 限制运动范围,以齿条为界
    if (target_pos > 130)
        target_pos = 130;

    if (target_pos < 20)
        target_pos =20;

    // 计算脉冲数
uint32_t pulses = (uint32_t)(fabsf((float)target_pos)
                             * LIFT_PULSE_PER_MM);

    // 1 上行，0 下行
    uint8_t dir = (target_pos > 0) ? 1 : 0;

    Emm_V5_Pos_Control(5, dir, speed, acc,
                       pulses, 1, false);
}

// 伸缩机构相对运动
void Tower_SetTelescope_Pos(int16_t target_pos,
                            uint16_t speed,
                            uint8_t acc)
{
    // 限制运动范围
    if (target_pos > 135)
        target_pos = 135;

    if (target_pos < -135)
        target_pos = -135;

    // 计算脉冲数
    uint32_t pulses =
        (uint32_t)(fabsf((float)target_pos) * 25.465f);

    // 0 伸出，1 缩回
    uint8_t dir = (target_pos > 0) ? 0 : 1;

    Emm_V5_Pos_Control(6, dir, speed, acc,
                       pulses, 0, false);
}


void Tower_Motor_Homing(void)
{
    // 伸缩机构回零
    Emm_V5_Origin_Trigger_Return(6, 2, false);

    // 升降机构回零
    Emm_V5_Origin_Trigger_Return(5, 2, false);
}