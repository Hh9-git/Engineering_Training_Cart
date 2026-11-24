#include "Chassis_task.h"
#include "Chassis_Odometry.h"
#include "dvc_stepmotor.h"
#include "dvc_yis320.h"
#include "drv_can.h"

uint8_t acc_vel=150;

chassis_t chassis = {0};   // 头文件里只有 extern 声明，定义放在这里


/* 正弦加减速起始速度 */
static float startVx = 0.0f;
static float startVy = 0.0f;


static void Chassis_PollEncoder(void);
static void Chassis_Accel_Update(void);

void Chassis_Init(void)
{
    AttachInterrupt_FDCAN(&hfdcan1, ZDT_CAN_Callback);

    chassis.accel_K = 50;
    chassis.ctrlMode = CHASSIS_LOCK_MODE;   // 上电即锁定，防止 yaw 扰动带动轮子

    PID_Init(&chassis.PID_angle, 2.0f, 0.0f, 0.0f, 100.0f, 20.0f);
    PID_Init(&chassis.PID_x, 0.5f, 0.0f, 0.0f, 150.0f, 20.0f);
    PID_Init(&chassis.PID_y, 0.5f, 0.0f, 0.0f, 150.0f, 20.0f);



    osDelay(500);
}

void Chassis_Task(void *argument)
{
    (void)argument;

    Chassis_Init();

    Chassis_Odometry_Init();
    Chassis_ResetPos();

    for (;;)
    {
        Chassis_PollEncoder();

        Chassis_Odometry_Update();

        // 将独立里程计模块的位置同步到底盘结构体
        chassis.Px = Chassis_Odometry_GetX();
        chassis.Py = Chassis_Odometry_GetY();

        Chassis_Update();

        osDelay(10);
    }
}

void Chassis_ResetPos(void)
{
    chassis.Px = 0.0f;
    chassis.Py = 0.0f;

    Chassis_Odometry_Reset();

    Chassis_clear();
}

static void Chassis_SetMotion(float Vx, float Vy,
                              float Yaw, float W,
                              uint8_t mode)
{
    // 保存当前速度，作为加速起点
    startVx = chassis.nowVx;
    startVy = chassis.nowVy;

    // 设置场地坐标系速度
    chassis.setVx = Vx;
    chassis.setVy = Vy;

    // 设置航向角和旋转速度
    chassis.setYaw = Yaw;
    chassis.setVw = W;

    // 重新开始加速规划
    chassis.accel_count = 0;
    chassis.accel_K = 50;

    // 设置运动模式
    chassis.ctrlMode = mode;

    // 清除 PID 历史状态
    Chassis_clear();
}

static void Chassis_PollEncoder(void)
{
    static const uint8_t k_wheel_addr[4] = { 1u, 2u, 3u, 4u };
    static uint8_t idx = 0u;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0u)
    {
        Emm_V5_Read_Sys_Params(k_wheel_addr[idx], S_CPOS);
        idx = (uint8_t)((idx + 1u) & 3u);
    }
}

static void Chassis_Accel_Update(void)
{
    if (chassis.accel_K == 0)
    {
        chassis.nowVx = chassis.setVx;
        chassis.nowVy = chassis.setVy;
        return;
    }

    if (chassis.accel_count < chassis.accel_K)
    {
        chassis.accel_count++;

        float t = 0.5f - 0.5f * cosf(
            CHASSIS_PI * (float)chassis.accel_count / chassis.accel_K
        );

        chassis.nowVx = startVx + (chassis.setVx - startVx) * t;
        chassis.nowVy = startVy + (chassis.setVy - startVy) * t;
    }
    else
    {
        chassis.nowVx = chassis.setVx;
        chassis.nowVy = chassis.setVy;
    }
}
//负责控制逻辑和速度计算
void Chassis_Update(void)
{
    /* 锁死模式：不读 yaw、不跑 PID，只每周期重发零速保持抱死 */
    if (chassis.ctrlMode == CHASSIS_LOCK_MODE)
    {
        Chassis_Move(0, 0, 0);
        return;
    }

    float yaw = IMU_getYaw();
    float theta = yaw * CHASSIS_PI / 180.0f;

    float Vx;
    float Vy;
    float W;
  if (chassis.ctrlMode == ROTATE_MODE)
    {
        float yaw_error = chassis.setYaw - yaw;

        // 将角度误差限制在 [-180, 180]
        while (yaw_error > 180.0f)
            yaw_error -= 360.0f;

        while (yaw_error < -180.0f)
            yaw_error += 360.0f;

        // 角度 PID
        W = PID_Calc(&chassis.PID_angle, 0.0f, yaw_error);

        // 到达目标角度后停止
        if (fabsf(yaw_error) < 0.5f)
        {
            W = 0.0f;
        }

        // 只旋转，不平移
        Chassis_Move(0, 0, (int16_t)W);

        return;
    }
    
// 位置模式
if (chassis.ctrlMode == POSITION_MODE)
{
    // 1. 计算场地坐标系位置误差
    // 目标位置 - 当前实际位置
    float error_x = chassis.setPx - chassis.Px;
    float error_y = chassis.setPy - chassis.Py;

    // 2. 位置 P 环：输出场地坐标系速度
    chassis.setVx = PID_Calc(&chassis.PID_x, 0.0f, error_x);
    chassis.setVy = PID_Calc(&chassis.PID_y, 0.0f, error_y);

    // 3. 到达目标位置后停止平移
    if (fabsf(error_x) < 5.0f &&
        fabsf(error_y) < 5.0f)
    {
        chassis.setVx = 0.0f;
        chassis.setVy = 0.0f;
    }

    // 4. 航向角误差
    float yaw_error = chassis.setYaw - yaw;

    while (yaw_error > 180.0f)
        yaw_error -= 360.0f;

    while (yaw_error < -180.0f)
        yaw_error += 360.0f;

    // 5. 航向 PID
    W = PID_Calc(&chassis.PID_angle, 0.0f, yaw_error);

    // 6. 速度规划
    Chassis_Accel_Update();

    // 7. 场地坐标系速度转换为底盘坐标系速度
    Vx = chassis.nowVx * cosf(theta)
       + chassis.nowVy * sinf(theta);

    Vy = -chassis.nowVx * sinf(theta)
       + chassis.nowVy * cosf(theta);

    // 8. 发送电机速度
    Chassis_Move((int16_t)Vx,
                 (int16_t)Vy,
                 (int16_t)W);

    return;
}

    // 以下保留你原来的速度控制逻辑
    Chassis_Accel_Update();

    Vx = chassis.nowVx * cosf(theta)
       + chassis.nowVy * sinf(theta);

    Vy = -chassis.nowVx * sinf(theta)
       + chassis.nowVy * cosf(theta);

    if (chassis.ctrlMode == VELOCITY_MODE)
    {
        float error = chassis.setYaw - yaw;

        while (error > 180.0f)
            error -= 360.0f;

        while (error < -180.0f)
            error += 360.0f;

        W = PID_Calc(&chassis.PID_angle, 0.0f, error);
    }
    else if (chassis.ctrlMode == SPIN_DRIFT_MODE)
    {
        W = chassis.setVw;
    }
    else
    {
        W = 0.0f;
    }

    Chassis_Move((int16_t)Vx,
                 (int16_t)Vy,
                 (int16_t)W);
}


/* 四轮速度分配：把底盘坐标系速度 (Vx, Vy, W) 拆成四个轮子的期望转速。
   speed[0..3] 与 Chassis_Motor_SetVel 里的电机地址 1~4 一一对应；
   各轮电气正方向的差异由 Chassis_Motor_SetVel 内部的 dir 处理，这里只管方向解算。 */
void Chassis_Move(int16_t Vx, int16_t Vy, int16_t W)
{
    int16_t speed[4];

    speed[0] = +Vx + Vy - W;
    speed[1] = -Vx + Vy + W;
    speed[2] = -Vx + Vy - W;
    speed[3] = +Vx + Vy + W;

    Chassis_Motor_SetVel(speed);
}


void Chassis_Motor_SetVel(int16_t speed[4])//此处用4，方便与电机对应1，2，3，4
{
    //正向
    if(speed[0]>=0){Emm_V5_Vel_Control(1, 0, speed[0], acc_vel, true);}else{Emm_V5_Vel_Control(1, 1, -speed[0], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[1]>=0){Emm_V5_Vel_Control(2, 1, speed[1], acc_vel, true);}else{Emm_V5_Vel_Control(2, 0, -speed[1], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[2]>=0){Emm_V5_Vel_Control(3, 0, speed[2], acc_vel, true);}else{Emm_V5_Vel_Control(3, 1, -speed[2], acc_vel, true);}
	osDelay(1);
    //正向
	if(speed[3]>=0){Emm_V5_Vel_Control(4, 1, speed[3], acc_vel, true);}else{Emm_V5_Vel_Control(4, 0, -speed[3], acc_vel, true);}
    osDelay(1);

    Emm_V5_Synchronous_motion(0); //广播触发
}



/**
 * @brief 场地坐标系速度控制，自动锁定当前航向
 * @param Vx 场地 X 方向速度，mm/s
 * @param Vy 场地 Y 方向速度，mm/s
 */
void Chassis_SetVel(float Vx, float Vy)
{
    Chassis_SetMotion(Vx, Vy,
                      IMU_getYaw(), 0.0f,
                      VELOCITY_MODE);
}
/**
 * 设置底盘目标位置
 * Px、Py：mm
 * Yaw：degree，逆时针为正
 */
void Chassis_SetPos(float Px, float Py, float Yaw)
{
    chassis.setPx = Px;
    chassis.setPy = Py;
    chassis.setYaw = Yaw;

    // 进入位置模式
    chassis.ctrlMode = POSITION_MODE;

    // 从当前规划速度开始加减速
    startVx = chassis.nowVx;
    startVy = chassis.nowVy;

    chassis.accel_count = 0;

    // 清除位置PID历史状态
    chassis.PID_x.err[0] = 0;
    chassis.PID_x.err[1] = 0;
    chassis.PID_x.Iout = 0;
    chassis.PID_x.out = 0;

    chassis.PID_y.err[0] = 0;
    chassis.PID_y.err[1] = 0;
    chassis.PID_y.Iout = 0;
    chassis.PID_y.out = 0;

    // 清除航向PID历史状态
    chassis.PID_angle.err[0] = 0;
    chassis.PID_angle.err[1] = 0;
    chassis.PID_angle.Iout = 0;
    chassis.PID_angle.out = 0;
}
void Chassis_clear(void)
{
    chassis.PID_x.err[0] = chassis.PID_x.err[1] = 0;
    chassis.PID_y.err[0] = chassis.PID_y.err[1] = 0;
    chassis.PID_angle.err[0] = chassis.PID_angle.err[1] = 0;
    chassis.PID_x.Iout = chassis.PID_x.out = 0;
    chassis.PID_y.Iout = chassis.PID_y.out = 0;
    chassis.PID_angle.Iout = chassis.PID_angle.out = 0;
}

/**
 * @brief 设置底盘旋转目标角度
 * @param target 目标角度，单位：degree
 *               逆时针为正
 */
void Chassis_Rotate(float target)
{
    // 设置目标角度
    chassis.setYaw = target;

    // 进入纯旋转模式
    chassis.ctrlMode = ROTATE_MODE;

    // 禁止平移
    chassis.setVx = 0.0f;
    chassis.setVy = 0.0f;
    chassis.nowVx = 0.0f;
    chassis.nowVy = 0.0f;

    // 清除 PID 历史状态
    Chassis_clear();
}

/**
 * @brief 锁死底盘：不跟航向、不平移，每个控制周期发送零速保持抱死
 * @note  需要恢复时直接调用 Chassis_SetVel / Chassis_SetPos / Chassis_Rotate 等，
 *        它们会改写 ctrlMode，加速起点从已清零的 nowV 开始。
 */
void Chassis_Lock(void)
{
    chassis.setVx = 0.0f;
    chassis.setVy = 0.0f;
    chassis.setVw = 0.0f;

    chassis.nowVx = 0.0f;
    chassis.nowVy = 0.0f;

    startVx = 0.0f;
    startVy = 0.0f;
    chassis.accel_count = 0;

    Chassis_clear();

    /* 最后写模式：被 Chassis 任务抢占时也只会是"旧模式 + 零目标" */
    chassis.ctrlMode = CHASSIS_LOCK_MODE;
}

/**
 * @brief 指定航向的平移控制
 * @param Vx 场地 X 方向速度，mm/s
 * @param Vy 场地 Y 方向速度，mm/s
 * @param Yaw 目标航向角，degree，逆时针为正
 */
void Chassis_Drift(float Vx, float Vy, float Yaw)
{
    Chassis_SetMotion(Vx, Vy,
                      Yaw, 0.0f,
                      VELOCITY_MODE);
}
/**
 * @brief 平移与持续旋转
 * @param Vx 场地 X 方向速度，mm/s
 * @param Vy 场地 Y 方向速度，mm/s
 * @param W 持续旋转控制量
 */
void Chassis_SpinDrift(float Vx, float Vy, float W)
{
    Chassis_SetMotion(Vx, Vy,
                      IMU_getYaw(), W,
                      SPIN_DRIFT_MODE);
}