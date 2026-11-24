
#include "dvc_servo.h"
#include "drv_pwm.h"

/* 舵机基本属性 */
Servo_t claw  = {0, 180, Set_Claw_Angle};
Servo_t gimbal = {0, 270, Set_Gimbal_Angle};
Servo_t tray  = {0, 270, Set_Tray_Angle};

/* 云台参数 */
#define GIMBAL_ANGLE_MAX    135
#define GIMBAL_INTERVAL     10U
#define GIMBAL_MOVE_TIME    100U

/* 云台运动对象 */
static Servo_Motion_t gimbal_motion =
{
    .start_angle = 0.0f,
    .current_angle = 0.0f,
    .target_angle = 0.0f,

    .start_tick = 0,

    .interval = GIMBAL_INTERVAL,
    .move_time = GIMBAL_MOVE_TIME,

    .moving_flag = 0
};

/* 内部函数声明 */
static void Servo_Motion_SetTarget(Servo_Motion_t *motion, float target);

static void Servo_Motion_Update(
    Servo_Motion_t *motion,
    void (*SetAngle)(int16_t)
);

static void Set_Gimbal_PWM(int16_t angle);


/* ==================== 初始化 ==================== */

void Servo_Init(void)
{
    PWM_Init();

    /* 初始化云台运动状态 */
    gimbal_motion.start_angle = 0.0f;
    gimbal_motion.current_angle = 0.0f;
    gimbal_motion.target_angle = 0.0f;
    gimbal_motion.moving_flag = 0;

    /* 输出各舵机初始角度 */
    Set_Claw_Angle(claw.start_angle);
    Set_Gimbal_PWM(0);
    Set_Tray_Angle(tray.start_angle);

    /* 开启舵机5V电源 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}


/* ==================== 夹爪舵机 ==================== */

void Set_Claw_Angle(int16_t angle)
{
    Servo_SetAngle_180(&htim2, TIM_CHANNEL_1, angle);
}


/* ==================== 云台舵机 ==================== */

/* 设置云台目标角度 */
void Set_Gimbal_Angle(int16_t angle)
{
    if (angle > GIMBAL_ANGLE_MAX)
        angle = GIMBAL_ANGLE_MAX;

    if (angle < -GIMBAL_ANGLE_MAX)
        angle = -GIMBAL_ANGLE_MAX;

    Servo_Motion_SetTarget(&gimbal_motion, (float)angle);
}

/* 设置运动目标 */
static void Servo_Motion_SetTarget(Servo_Motion_t *motion, float target)
{
    /* 从当前插值角度开始 */
    motion->start_angle = motion->current_angle;
    motion->target_angle = target;

    motion->start_tick = HAL_GetTick();

    motion->moving_flag = 1;
}

/* 通用五次多项式S曲线更新 */
static void Servo_Motion_Update(
    Servo_Motion_t *motion,
    void (*SetAngle)(int16_t))
{
    uint32_t now = HAL_GetTick();

    if (!motion->moving_flag)
        return;

    /* 按实际时间计算运动进度 */
    float tau = (float)(now - motion->start_tick)
                / motion->move_time;

    if (tau >= 1.0f)
    {
        motion->current_angle = motion->target_angle;
        motion->moving_flag = 0;
    }
    else
    {
        /* 五次多项式S曲线 */
        float t2 = tau * tau;
        float t3 = t2 * tau;
        float t4 = t3 * tau;
        float t5 = t4 * tau;

        float s = 6.0f * t5
                - 15.0f * t4
                + 10.0f * t3;

        motion->current_angle =
            motion->start_angle
            + (motion->target_angle - motion->start_angle) * s;
    }

    SetAngle((int16_t)motion->current_angle);
}

/* 云台PWM输出 */
static void Set_Gimbal_PWM(int16_t angle)
{
    Servo_SetAngle_270(&htim2, TIM_CHANNEL_3, angle);
}

/* 云台周期更新 */
void Servo_Gimbal_Update(void)
{
    static uint32_t last_update_tick = 0;

    uint32_t now = HAL_GetTick();

    if (now - last_update_tick < gimbal_motion.interval)
        return;

    last_update_tick = now;

    Servo_Motion_Update(&gimbal_motion, Set_Gimbal_PWM);
}


/* ==================== 转盘舵机 ==================== */

void Set_Tray_Angle(int16_t angle)
{
    Servo_SetAngle_270(&htim1, TIM_CHANNEL_1, angle);
}


/* ==================== PWM底层输出 ==================== */

void Servo_SetAngle_270(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    int16_t angle)
{
    if (angle > 135)
        angle = 135;

    if (angle < -135)
        angle = -135;

    /* -135°=500us，0°=1500us，+135°=2500us */
    uint32_t ccr = 1500 + (angle * 1000) / 135;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


void Servo_SetAngle_180(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    int16_t angle)
{
    if (angle > 90)
        angle = 90;

    if (angle < -90)
        angle = -90;

    uint32_t ccr = 1500 + (angle * 1000) / 90;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


void Servo_SetPulse(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint16_t us)
{
    if (us < 500)
        us = 500;

    if (us > 2500)
        us = 2500;

    __HAL_TIM_SET_COMPARE(htim, channel, us);
}
