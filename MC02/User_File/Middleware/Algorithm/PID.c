#include "PID.h"


PID mypid = {0}; //创建一个PID结构体变量

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    PID_Reset(pid);
}


/**
  * @brief  清零状态量，保留增益和限幅。重新开始时调用，防止旧积分残留。
  */
void PID_Reset(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->lastFeedback = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}



/**
  * @brief  进行一次pid计算
  *         参数为(pid结构体,目标值,反馈值)
  *
  * @note   reference/feedback 为角度制，误差按 ±180° 取最短路径。
  *         微分项对反馈量微分，目标值突变时不会产生微分冲击。
  *
  * @param  pid
  * @param  reference: target_of_value
  * @param  feedback
  * @retval None
  */
void PID_Calc(PID *pid, float reference, float feedback)
{
 	  //更新数据
    pid->lastError = pid->error; //将旧error存起来
    //计算新误差
    if(reference - feedback < -180){pid->error = reference - feedback + 360;}
    else if(reference - feedback > 180){pid->error = reference - feedback - 360;}
    else{pid->error = reference - feedback;}
    //计算微分（对反馈量微分，负号来自 e = ref - fb）
    float dout = (pid->lastFeedback - feedback) * pid->kd;
    pid->lastFeedback = feedback;
    //计算比例
    float pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout+dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}