
#ifndef ALG_PID_H
#define ALG_PID_H


#include "main.h"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float err[2]; // 误差及上一次误差
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
} PID_t; // PID

void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout);
float PID_Calc(PID_t *pid, float fdb, float set);

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float max_out;  // 最大输出
    float max_iout; // 最大积分输出

    float err[2]; // 误差及上一次误差
    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;

    float h;  // 快速跟踪因子
    float r;  // 系统调用步长
    float x1; // 输入
    float x2; // 输入的微分
} TDPID_t;    // TDPID
void TDPID_Init(TDPID_t *TDpid, float kp, float ki, float kd, float max_out, float max_iout, float h, float r);
float TDPID_Calc(TDPID_t *TDpid, float fdb, float set);

typedef struct
{
    float A0;       /**< The derived gain, A0 = Kp + Ki + Kd . */
    float A1;       /**< The derived gain, A1 = -Kp - 2Kd. */
    float A2;       /**< The derived gain, A2 = Kd . */
    float state[2]; /**< The state array of length 3. */
    float Kp;       /**< The proportional gain. */
    float Ki;       /**< The integral gain. */
    float Kd;       /**< The derivative gain. */
    float MaxOut;   /**< The Max output. */
    float out;
} fastPID_t; // 快速计算PID

void FastPID_Init(fastPID_t *S, float kp, float ki, float kd, float maxout);
float FastPID_Calc(fastPID_t *S, float in);

typedef struct
{
    float Kp;
    float Kd;
    float dp;
    float dv;
    float t;
    float out;
} Impedance_t; // 阻抗控制

/* 阻抗控制计算 */
static inline float Impedance_Calc(Impedance_t *imp)
{
    imp->out = imp->Kp * imp->dp + imp->Kd * imp->dv + imp->t;
    return imp->out;
}


#endif