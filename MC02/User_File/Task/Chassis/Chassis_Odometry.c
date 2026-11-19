
#include "Chassis_Odometry.h"

#include "dvc_stepmotor.h"
#include "dvc_yis320.h"



static int32_t motor_last_pos[4] = {0};
static uint32_t motor_last_seq[4] = {0};

static const int8_t motor_pos_dir[4] =
{
    +1, +1, +1, +1
};

static Chassis_Odometry_t odom = {0};

void Chassis_Odometry_Init(void)
{
    odom.Px = 0.0f;
    odom.Py = 0.0f;
    odom.initialized = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        motor_last_pos[i] = 0;
        motor_last_seq[i] = 0;
    }
}

void Chassis_Odometry_Reset(void)
{
    odom.Px = 0.0f;
    odom.Py = 0.0f;
    odom.initialized = 0;
}

void Chassis_Odometry_Update(void)
{
    int32_t pos[4];
    uint32_t seq[4];

    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t id = i + 1;

        if (!Emm_Feedback[id].valid)
            return;

        pos[i] = Emm_Feedback[id].pos_raw;
        seq[i] = Emm_Feedback[id].seq;

        if ((HAL_GetTick() - Emm_Feedback[id].tick) > 100)
            return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (seq[i] == motor_last_seq[i])
            return;
    }

    // 第一次只记录编码器初始位置
    if (!odom.initialized)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            motor_last_pos[i] = pos[i];
            motor_last_seq[i] = seq[i];
        }

        odom.initialized = 1;
        return;
    }

    // 计算四轮位移
    for (uint8_t i = 0; i < 4; i++)
    {
        int64_t delta = (int64_t)pos[i] - motor_last_pos[i];

        odom.wheel_ds[i] =
            (float)delta * CHASSIS_POS_TO_MM * motor_pos_dir[i];

        motor_last_pos[i] = pos[i];
        motor_last_seq[i] = seq[i];
    }

    // 注意：这里暂时保留你原来的逆解公式，
    // 后续需要通过实测确认其与 Chassis_Move() 一致。
    odom.dx_body =
        (odom.wheel_ds[0] + odom.wheel_ds[1]
       - odom.wheel_ds[2] - odom.wheel_ds[3]) * 0.25f;

    odom.dy_body =
        (odom.wheel_ds[0] - odom.wheel_ds[1]
       + odom.wheel_ds[2] - odom.wheel_ds[3]) * 0.25f;

    float yaw = IMU_getYaw() * CHASSIS_PI / 180.0f;

    odom.dx_field =
        odom.dx_body * cosf(yaw) - odom.dy_body * sinf(yaw);

    odom.dy_field =
        odom.dx_body * sinf(yaw) + odom.dy_body * cosf(yaw);

    odom.Px += odom.dx_field;
    odom.Py += odom.dy_field;
}

float Chassis_Odometry_GetX(void)
{
    return odom.Px;
}

float Chassis_Odometry_GetY(void)
{
    return odom.Py;
}
