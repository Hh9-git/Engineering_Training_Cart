#include "alg_pid.h"


// PID初始化
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

/* PID计算, 输入反馈值和预期值 */
float PID_Calc(PID_t *pid, float fdb, float set)
{
    pid->set = set;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->set - pid->fdb;

    pid->Pout = pid->Kp * pid->err[0];
    pid->Iout += pid->Ki * pid->err[0];
    pid->Dout = pid->Kd * (pid->err[0] - pid->err[1]);

    if (pid->Iout > pid->max_iout)
    {
        pid->Iout = pid->max_iout;
    }
    else if (pid->Iout < -pid->max_iout)
    {
        pid->Iout = -pid->max_iout;
    }

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    else if (pid->out < -pid->max_out)
    {
        pid->out = -pid->max_out;
    }

    return pid->out;
}

/*****************************sign函数*********************************/
static inline float sign(float x)
{
    if (x > 0)
        return 1;
    else if (x < 0)
        return -1;
    else
        return 0;
}

static inline float fhan(float x1, float x2, float r, float h)
{
    float d, d0, y, a0, a;

    d = r * h;
    d0 = d * h;
    y = x1 + x2 * h;
    a0 = sqrtf(d * d + 8 * r * fabsf(y));
    if (fabsf(y) <= d0)
        a = x2 + y / h;
    else
        a = x2 + 0.5 * (a0 - d) * sign(y);

    if (fabsf(a) <= d)
        return -r * a / d;
    else
        return -r * sign(a);
}

void TDPID_Init(TDPID_t *TDpid, float kp, float ki, float kd, float max_out, float max_iout, float h, float r)
{
    TDpid->Kp = kp;
    TDpid->Ki = ki;
    TDpid->Kd = kd;
    TDpid->max_out = max_out;
    TDpid->max_iout = max_iout;
    TDpid->h = h;
    TDpid->r = r;
    TDpid->err[0] = TDpid->err[1] = 0.0f;
}

/* TDPID */
float TDPID_Calc(TDPID_t *TDpid, float fdb, float set)
{
    /******************************TD****************************************/
    TDpid->x1 += TDpid->h * TDpid->x2;
    TDpid->x2 += TDpid->h * fhan(TDpid->x1 - set, TDpid->x2, TDpid->r, TDpid->h);

    TDpid->set = set;
    TDpid->fdb = fdb;
    TDpid->err[1] = TDpid->err[0];
    TDpid->err[0] = TDpid->x1 - TDpid->fdb;

    TDpid->Pout = TDpid->Kp * TDpid->err[0];
    TDpid->Iout += TDpid->Ki * TDpid->err[0];
    TDpid->Dout = TDpid->Kd * (TDpid->err[0] - TDpid->err[1]);

    if (TDpid->Iout > TDpid->max_iout)
    {
        TDpid->Iout = TDpid->max_iout;
    }
    else if (TDpid->Iout < -TDpid->max_iout)
    {
        TDpid->Iout = -TDpid->max_iout;
    }

    TDpid->out = TDpid->Pout + TDpid->Iout + TDpid->Dout;

    if (TDpid->out > TDpid->max_out)
    {
        TDpid->out = TDpid->max_out;
    }
    else if (TDpid->out < -TDpid->max_out)
    {
        TDpid->out = -TDpid->max_out;
    }

    return TDpid->out;
}
void FastPID_Init(fastPID_t *S, float kp, float ki, float kd, float maxout)
{
    S->A0 = kp + ki + kd;
    S->A1 = -kp - 2 * kd;
    S->A2 = kd;
    S->MaxOut = maxout;
    S->state[0] = 0.0f;
    S->state[1] = 0.0f;
    S->out = 0.0f;
}

/* 增量式PID计算, 输入in为误差值 */
float FastPID_Calc(fastPID_t *S, float in)
{
    /* detPID = Kp*(Error_1(k)-Error_1(k-1)) + Ki*(Error_1(k)) + Kd*(Error_1(k)-2*Error_2(k-1)+Error_2(k-2)) */
    S->out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->out);

    /* Update state */
    S->state[1] = S->state[0];
    S->state[0] = in;

    if (S->out > S->MaxOut)
    {
        return (S->MaxOut);
    }
    else if (S->out < -S->MaxOut)
    {
        return -(S->MaxOut);
    }

    return (S->out);
}