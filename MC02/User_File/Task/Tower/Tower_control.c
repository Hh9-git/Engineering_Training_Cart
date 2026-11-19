
#include "Tower_control.h"

#include "dvc_stepmotor.h"
#include "dvc_servo.h"

#include <math.h>

void Tower_Control_Init(void)
{
    Servo_Init();
}

// 云台角度控制
void Tower_SetGimbal_Angle(int16_t angle)
{
    Set_Gimbal_Angle(angle);
}

// 升降机构相对运动
void Tower_SetLift_Pos(int16_t target_pos,
                       uint16_t speed,
                       uint8_t acc)
{
    // 限制运动范围
    if (target_pos > 135)
        target_pos = 135;

    if (target_pos < -135)
        target_pos = -135;

    // 计算脉冲数
    uint32_t pulses = (uint32_t)(fabsf((float)target_pos) * 80.0f);

    // 1 上行，0 下行
    uint8_t dir = (target_pos > 0) ? 1 : 0;

    Emm_V5_Pos_Control(5, dir, speed, acc,
                       pulses, 0, false);
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
