#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "main.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

extern float imuAngle[3];   // IMU_task 里持续更新的欧拉角 yaw/pitch/roll，单位 rad
extern float gyro[3];       // 陀螺原始角速度，单位 rad/s（未减零偏，供 LCD 交叉验证）
extern float gyro_cal[3];   // 减掉零偏后喂给 AHRS 的角速度，单位 rad/s
extern float gyro_bias[3];  // 标定得到的陀螺零偏，单位 rad/s
extern float acc[3];        // 加速度，单位 m/s^2
extern float imuTemp;       // 传感器温度，单位 ℃
extern uint8_t imu_calibrated;  // 0=正在标定 1=标定完成


#endif
