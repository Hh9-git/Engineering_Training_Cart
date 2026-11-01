#include "alg_vqf.h"
#include "vqf.hpp"

#include <new>

/* 必须有这个宏，否则 vqf_real_t 是 double，与 C 侧 float 缓冲区 ABI 不一致 */
#ifndef VQF_SINGLE_PRECISION
#error "VQF_SINGLE_PRECISION must be defined (target_compile_definitions)"
#endif

static_assert(sizeof(vqf_real_t) == sizeof(float),
              "VQF_SINGLE_PRECISION must be consistent in every translation unit");

#define GRAVITY_MS2        9.80665f
#define MOTION_ACC_TAU_S   0.1f     /* 运动加速度低通时间常数 */

/* VQF 内部只用固定数组，无堆分配；用静态缓冲区 + placement new，
   避免依赖 .init_array 的全局构造顺序。 */
alignas(VQF) static unsigned char s_vqf_mem[sizeof(VQF)];
static VQF *s_vqf = nullptr;

static float s_quat[4]       = {1.0f, 0.0f, 0.0f, 0.0f};
static float s_world_acc[3]  = {0.0f, 0.0f, 0.0f};
static float s_motion_acc[3] = {0.0f, 0.0f, 0.0f};
static float s_motion_alpha  = 0.0f;

static void ResetOutputs(void)
{
    s_quat[0] = 1.0f;
    s_quat[1] = 0.0f;
    s_quat[2] = 0.0f;
    s_quat[3] = 0.0f;
    s_world_acc[0]  = s_world_acc[1]  = s_world_acc[2]  = 0.0f;
    s_motion_acc[0] = s_motion_acc[1] = s_motion_acc[2] = 0.0f;
}

void VQF_Init(float gyr_ts)
{
    if (s_vqf != nullptr)
    {
        s_vqf->~VQF();
        s_vqf = nullptr;
    }

    s_vqf = new (s_vqf_mem) VQF((vqf_real_t)gyr_ts);
    s_vqf->resetState();

    /* y += alpha * (x - y) */
    s_motion_alpha = gyr_ts / (MOTION_ACC_TAU_S + gyr_ts);

    ResetOutputs();
}

void VQF_Reset(void)
{
    if (s_vqf != nullptr)
    {
        s_vqf->resetState();
    }
    ResetOutputs();
}

void VQF_Update(const float gyr[3], const float acc[3])
{
    if (s_vqf == nullptr)
    {
        return;
    }

    vqf_real_t g[3] = {gyr[0], gyr[1], gyr[2]};
    vqf_real_t a[3] = {acc[0], acc[1], acc[2]};

    s_vqf->update(g, a);

    vqf_real_t q[4];
    s_vqf->getQuat6D(q);
    for (int i = 0; i < 4; i++)
    {
        s_quat[i] = (float)q[i];
    }

    /* 世界系加速度（VQF 约定：地系 z 向上，静止时 R*a = [0,0,+g]） */
    vqf_real_t world[3];
    VQF::quatRotate(q, a, world);
    for (int i = 0; i < 3; i++)
    {
        s_world_acc[i] = (float)world[i];
    }

    /* 去重力后再低通；每个采样更新一次，不在 getter 里更新。
       注意：VQF 用加速度计确定重力方向，持续的直线加速度会被缓慢当成倾斜吸收，
       因此这里得到的是“瞬态”运动加速度（时间尺度由 tauAcc 决定）。 */
    vqf_real_t motion[3] = {world[0], world[1], world[2] - (vqf_real_t)GRAVITY_MS2};
    for (int i = 0; i < 3; i++)
    {
        s_motion_acc[i] += s_motion_alpha * ((float)motion[i] - s_motion_acc[i]);
    }
}

void VQF_GetQuat(float q[4])
{
    for (int i = 0; i < 4; i++)
    {
        q[i] = s_quat[i];
    }
}

void VQF_GetWorldAcc(float acc[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i] = s_world_acc[i];
    }
}

void VQF_GetMotionAcc(float acc[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i] = s_motion_acc[i];
    }
}

float VQF_GetBias(float bias[3])
{
    if (s_vqf == nullptr)
    {
        bias[0] = bias[1] = bias[2] = 0.0f;
        return -1.0f;
    }

    vqf_real_t b[3] = {0.0f, 0.0f, 0.0f};
    vqf_real_t sigma = s_vqf->getBiasEstimate(b);

    for (int i = 0; i < 3; i++)
    {
        bias[i] = (float)b[i];
    }
    return (float)sigma;
}

uint8_t VQF_RestDetected(void)
{
    return (s_vqf != nullptr && s_vqf->getRestDetected()) ? 1u : 0u;
}