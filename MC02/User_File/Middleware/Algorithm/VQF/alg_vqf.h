#ifndef ALG_VQF_H
#define ALG_VQF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 采样周期必须与实际调用 VQF_Update 的频率一致（VQF 内部按固定 dt 积分） */
void    VQF_Init(float gyr_ts);
void    VQF_Reset(void);

/* gyr: rad/s（未减零偏，VQF 内部估计）；acc: m/s²（含重力） */
void    VQF_Update(const float gyr[3], const float acc[3]);

/* 四元数 [w,x,y,z]，机体系 -> 世界系 */
void    VQF_GetQuat(float q[4]);

/* 世界系总加速度 m/s²（含重力） */
void    VQF_GetWorldAcc(float acc[3]);

/* 世界系运动加速度 m/s²（去重力 + 一阶低通） */
void    VQF_GetMotionAcc(float acc[3]);

/* 陀螺零偏 rad/s；返回零偏估计标准差 sigma（rad/s），尚未初始化时返回 -1 */
float   VQF_GetBias(float bias[3]);

/* 1 = VQF 判定当前静止 */
uint8_t VQF_RestDetected(void);

#ifdef __cplusplus
}
#endif

#endif /* ALG_VQF_H */