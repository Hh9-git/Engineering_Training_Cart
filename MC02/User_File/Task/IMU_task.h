#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "main.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define RAD_TO_DEG  57.29578f   /* 180/π，imuAngle[] 为 rad，显示前乘它转成 deg */

extern float imuAngle[3];   // IMU_task 里持续更新的欧拉角 yaw/pitch/roll，单位 rad
extern float imuQuat[4];    // VQF 四元数 [w,x,y,z]，机体系 -> 世界系
extern float gyro[3];       // 陀螺原始角速度，单位 rad/s（未减零偏）
extern float gyro_cal[3];   // 原始角速度减去 VQF 零偏估计，单位 rad/s
extern float gyro_bias[3];  // VQF 估计的陀螺零偏，单位 rad/s
extern float acc[3];        // 机体系加速度，单位 m/s^2（含重力）
extern float imuWorldAcc[3];   // 世界系总加速度，单位 m/s^2（含重力）
extern float imuMotionAcc[3];  // 世界系运动加速度，单位 m/s^2（去重力 + 低通）
extern float imuTemp;       // 传感器温度，单位 ℃
extern float imu_bias_sigma;    // 零偏估计标准差，单位 rad/s
extern float imu_sample_rate_hz; // DWT 实测采样率，单位 Hz
extern float imu_heater_out;     // 加热器 PWM 输出 0~500（诊断用）
extern uint8_t imu_calibrated;   // 1 = VQF 初始化完成（LCD 显示 OK）
extern uint8_t imu_rest_detected; // 1 = VQF 判定当前静止


#endif
