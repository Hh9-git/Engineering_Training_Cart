#include "IMU_task.h"
#include "BMI088driver.h"
#include "alg_vqf.h"
#include "bsp_dwt.h"
#include <math.h>

/* 与 BMI088 陀螺 ODR (1000 Hz) 一致；改这里时 VQF_GYR_TS 自动跟随 */
#define IMU_RATE_HZ        1000u
#define IMU_PERIOD_TICKS   ((uint32_t)(configTICK_RATE_HZ / IMU_RATE_HZ))
#define VQF_GYR_TS         (1.0f / (float)IMU_RATE_HZ)

/* 加热温控：设定值必须高于板子自然平衡温度，否则加热器永远不工作 */
#define IMU_HEATER_TARGET_C   35.0f
#define HEATER_KP             100.0f
#define HEATER_KI             50.0f
#define HEATER_KD             10.0f
#define HEATER_MAX_OUT        500u    /* TIM3: PSC=23, ARR=9999, 1 kHz -> 5% 稳态最大占空比 */

/* 升温 boost：只用于冷启动阶段，温度进入目标附近或超时后永久切回 PID */
#define HEATER_BOOST_OUT        1000u  /* 10% 占空比 */
#define HEATER_BOOST_MARGIN_C   2.0f   /* 距目标 2℃ 内结束 boost */
#define HEATER_BOOST_TIMEOUT_MS 60000u /* 最多 boost 60s，防止目标不可达时持续高功率 */
#define HEATER_CUTOFF_C         45.0f  /* 超温保护：高于此值强制关闭 */

float gyro[3]       = {0.0f};
float gyro_cal[3]   = {0.0f};
float gyro_bias[3]  = {0.0f};
float acc[3]        = {0.0f};
float imuTemp       = 0.0f;

float imuQuat[4]        = {1.0f, 0.0f, 0.0f, 0.0f};
float imuAngle[3]       = {0.0f};
float imuWorldAcc[3]    = {0.0f};
float imuMotionAcc[3]   = {0.0f};
float imu_bias_sigma    = -1.0f;
float imu_sample_rate_hz = 0.0f;
float imu_heater_out    = 0.0f;

uint8_t imu_calibrated    = 0;
uint8_t imu_rest_detected = 0;

static uint32_t s_dwt_last = 0;
static uint32_t s_rate_cnt = 0;
static float    s_rate_time = 0.0f;

static float s_heat_err   = 0.0f;
static float s_heat_err_l = 0.0f;
static float s_heat_err_ll = 0.0f;
static uint8_t  s_heat_warmup = 1;   /* 1 = 还在冷启动升温阶段 */
static uint32_t s_heat_start  = 0;   /* boost 起始 tick，用于超时 */

/* ZYX 欧拉角，q = [w,x,y,z]，与旧 Mahony 实现保持一致 */
static void GetAngle(const float q[4], float *yaw, float *pitch, float *roll)
{
    float sinp = 2.0f * (q[0] * q[2] - q[1] * q[3]);
    if (sinp > 1.0f)  sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    *pitch = asinf(sinp);
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

/* 1 秒窗口统计实际采样率，用于核对 VQF 的固定 dt 是否与真实循环周期一致 */
static void ImuRateStatUpdate(void)
{
    float dt = DWT_GetDeltaT(&s_dwt_last);
    if (dt <= 0.0f) return;

    s_rate_cnt++;
    s_rate_time += dt;

    if (s_rate_time >= 1.0f)
    {
        imu_sample_rate_hz = (float)s_rate_cnt / s_rate_time;
        s_rate_cnt  = 0;
        s_rate_time = 0.0f;
    }
}

void IMU_Task(void *argument)
{
    (void)argument;

    osDelay(10);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* 加热器 PB1 / TIM3_CH4 */

    while (BMI088_init() != BMI088_NO_ERROR)
    {
        osDelay(100);
    }

    VQF_Init(VQF_GYR_TS);
    imu_calibrated = 1;     /* 无开机标定流程，解算器就绪即显示 OK */

    s_dwt_last = DWT->CYCCNT;
    s_heat_start = osKernelGetTickCount();
    uint32_t wake = osKernelGetTickCount() + IMU_PERIOD_TICKS;

    for (;;)
    {
        BMI088_read(gyro, acc, &imuTemp);   /* rad/s, m/s², ℃ */

        /* 加热：冷启动阶段先用 10% 占空比快升，进入目标 2℃ 内或超时后切回 PD */
        s_heat_err_ll = s_heat_err_l;
        s_heat_err_l  = s_heat_err;
        s_heat_err    = IMU_HEATER_TARGET_C - imuTemp;

        uint32_t heat_now = osKernelGetTickCount();
        if (s_heat_warmup &&
            (imuTemp >= IMU_HEATER_TARGET_C - HEATER_BOOST_MARGIN_C ||
             (heat_now - s_heat_start) >= HEATER_BOOST_TIMEOUT_MS))
        {
            s_heat_warmup = 0;
        }

        float heat_out;
        if (!isfinite(imuTemp) || imuTemp > HEATER_CUTOFF_C)
        {
            /* 超温或温度读数异常：强制关闭 */
            heat_out = 0.0f;
        }
        else if (s_heat_warmup)
        {
            heat_out = (float)HEATER_BOOST_OUT;
        }
        else
        {
            heat_out = HEATER_KP * s_heat_err
                     + HEATER_KI * (s_heat_err + s_heat_err_l + s_heat_err_ll)
                     + HEATER_KD * (s_heat_err - s_heat_err_l);

            if (heat_out > (float)HEATER_MAX_OUT) heat_out = (float)HEATER_MAX_OUT;
            if (heat_out < 0.0f) heat_out = 0.0f;
        }

        imu_heater_out = heat_out;
        htim3.Instance->CCR4 = (uint16_t)heat_out;

        VQF_Update(gyro, acc);

        VQF_GetQuat(imuQuat);
        GetAngle(imuQuat, &imuAngle[INS_YAW_ADDRESS_OFFSET],
                          &imuAngle[INS_PITCH_ADDRESS_OFFSET],
                          &imuAngle[INS_ROLL_ADDRESS_OFFSET]);

        VQF_GetWorldAcc(imuWorldAcc);
        VQF_GetMotionAcc(imuMotionAcc);

        imu_bias_sigma    = VQF_GetBias(gyro_bias);
        imu_rest_detected = VQF_RestDetected();

        /* 兼容旧观测量：VQF 内部已经减零偏，这里只用于显示/交叉验证 */
        for (int i = 0; i < 3; i++)
        {
            gyro_cal[i] = gyro[i] - gyro_bias[i];
        }

        ImuRateStatUpdate();

        /* osDelayUntil 用绝对 tick；迟到返回错误时必须重对齐，否则会忙等 */
        if (osDelayUntil(wake) != osOK)
        {
            wake = osKernelGetTickCount() + IMU_PERIOD_TICKS;
        }
        else
        {
            wake += IMU_PERIOD_TICKS;
        }
    }
}