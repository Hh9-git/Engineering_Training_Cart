#include "IMU_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
#include "bsp_buzzer.h"
#include "dvc_lcd.h"
#include <math.h>


#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

/* 陀螺零偏标定：PD 加热器把传感器稳定到 DES_TEMP 后再采一段静止数据求均值。
   零偏随温度漂移，所以不能在开机瞬间(室温)标定，必须等温度到位。 */
#define CALI_TEMP_TOL      1.0f     // 温度进入目标 ±1℃ 视为稳定（PD 是振荡的，带宽放宽）
#define CALI_SETTLE_TICKS  2000     // 累计稳定 2s 后才开始采样
#define CALI_SAMPLE_NUM    2000     // 采样 2000 次，1kHz 下约 2s
#define CALI_ABORT_LIMIT   0.05f    // 采样期间任一分量超此值视为被晃动，作废重来
#define CALI_TIMEOUT_TICKS 60000    // 等 60s 还没稳就强制标定，防止加热器够不到 DES_TEMP 时永不标定

float gyro[3] = {0.0f};
float gyro_cal[3] = {0.0f};
float gyro_bias[3] = {0.0f};
float acc[3] = {0.0f};
float imuTemp = 0.0f;
uint8_t imu_calibrated = 0;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void IMU_Task(void *argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);

    uint32_t settle_cnt = 0;    // 温度累计落在容差带内的 tick 数
    uint32_t total_cnt  = 0;    // 上电以来的 tick 数，用于超时
    uint32_t sample_cnt = 0;    // 已累计的标定样本数
    float sum[3] = {0.0f};

    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &imuTemp);

        /* 加热 PD：标定等待期间也必须每轮都跑，否则温度下落、err 永远进不了容差带 */
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - imuTemp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
        htim3.Instance->CCR4 = (uint16_t)out;

        if (!imu_calibrated)
        {
            total_cnt++;

            /* 用累计(而非连续)计数，避免 PD 振荡把计数反复清零导致永远不达标 */
            if (fabsf(err) < CALI_TEMP_TOL)
            {
                settle_cnt++;
            }

            /* 温度稳定足够久，或等待超时，才允许开始采样 */
            if (settle_cnt >= CALI_SETTLE_TICKS || total_cnt >= CALI_TIMEOUT_TICKS)
            {
                if (fabsf(gyro[0]) > CALI_ABORT_LIMIT ||
                    fabsf(gyro[1]) > CALI_ABORT_LIMIT ||
                    fabsf(gyro[2]) > CALI_ABORT_LIMIT)
                {
                    /* 板子被晃动了，这段数据作废，重新采 */
                    sum[0] = sum[1] = sum[2] = 0.0f;
                    sample_cnt = 0;
                }
                else
                {
                    sum[0] += gyro[0];
                    sum[1] += gyro[1];
                    sum[2] += gyro[2];
                    sample_cnt++;

                    if (sample_cnt >= CALI_SAMPLE_NUM)
                    {
                        gyro_bias[0] = sum[0] / (float)sample_cnt;
                        gyro_bias[1] = sum[1] / (float)sample_cnt;
                        gyro_bias[2] = sum[2] / (float)sample_cnt;
                        imu_calibrated = 1;

                        /* 丢掉预热期间累积的 yaw，从标定完成这一刻重新积分 */
                        AHRS_init(imuQuat);
                        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);

                        /* 标定完成提示音。会阻塞约 0.33s：这期间不读传感器也不跑 PD，
                         * 但 AHRS 同样没在积分，所以不会凭空累积 yaw；
                         * 加热器停在稳态输出上，温度也不会掉。 */
                        Buzzer_CalibrationDoneSound();
                    }
                }
            }

            osDelay(1);
            continue;
        }

        /* 减掉零偏再喂 AHRS。gyro[] 保留原始值不覆盖，LCD 才能交叉验证零偏 */
        gyro_cal[0] = gyro[0] - gyro_bias[0];
        gyro_cal[1] = gyro[1] - gyro_bias[1];
        gyro_cal[2] = gyro[2] - gyro_bias[2];

        AHRS_update(imuQuat, gyro_cal, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);

        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}



