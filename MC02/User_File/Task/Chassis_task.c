#include "Chassis_task.h"
#include "dvc_stepmotor.h"
#include "bsp_ws2812.h"
#include "alg_fastmath.h"
#include "IMU_task.h"
#include "dvc_yis320.h"
// #include "task.h"          // taskENTER_CRITICAL / taskEXIT_CRITICAL
#include <math.h>
#include <stdint.h>

/** 
        底盘移动坐标系    W方向遵循坐标系右手法则
        ^y
        |                   
        |______>x          


        底盘俯视图
        
速度解算俯视图
 Speed[1]          Speed[2]                  
    \               /
      \           /
        \       /
          \   /
            *
          /    \
        /        \
      /            \
    /                \
Speed[3]           Speed[4]


        
实际底盘电机ID俯视图
 0x01               0x02                |
    \               /                   |
      \           /                     |
        \       /   {  }                   
          \   /    {tower}{claw}
            *       {  }
          /    \
        /        \
      /            \
    /     {tray}     \
0x03                0x04

*/

uint8_t acc_vel=150;

chassis_t chassis = {0};   // 头文件里只有 extern 声明，定义放在这里


/* 正弦加减速起始速度 */
static float startVx = 0.0f;
static float startVy = 0.0f;


static int32_t motor_last_pos[4] = {0};
static uint32_t motor_last_seq[4] = {0};

static uint8_t odometry_init = 0;
// 如果反馈正方向与 Chassis_Move() 中对应轮子的正方向相反，将该项改成 -1
static const int8_t motor_pos_dir[4] =
{
    +1, +1, +1, +1
};

static void Chassis_PollEncoder(void);
static void Chassis_Accel_Update(void);

/*调试用*/
volatile float debug_wheel_ds[4] = {0};

volatile float debug_dx_body = 0.0f;
volatile float debug_dy_body = 0.0f;

volatile float debug_dx_field = 0.0f;
volatile float debug_dy_field = 0.0f;

volatile float debug_start_yaw = 0.0f;
volatile float debug_target_yaw = 0.0f;
volatile float debug_current_yaw = 0.0f;
volatile float debug_yaw_error = 0.0f;

void Chassis_Init(void)
{
    AttachInterrupt_FDCAN(&hfdcan1, ZDT_CAN_Callback);

    chassis.accel_K = 50;
    chassis.ctrlMode = VELOCITY_MODE;

    PID_Init(&chassis.PID_angle, 2.0f, 0.0f, 0.0f, 100.0f, 20.0f);

    PID_Init(&chassis.PID_x, 0.5f, 0.0f, 0.0f, 150.0f, 20.0f);
    PID_Init(&chassis.PID_y, 0.5f, 0.0f, 0.0f, 150.0f, 20.0f);

    osDelay(500);
}

void Chassis_Task(void *argument)
{
    (void)argument;

    Chassis_Init();
    Chassis_ResetPos();
    

    Chassis_Rotate(90.0f);
    for (;;)
    {
        Chassis_PollEncoder();
        Chassis_Odometry_Update();
        Chassis_Update();

        osDelay(10);
    }
}

void Chassis_Odometry_Update(void)
{
    int32_t pos[4];
    uint32_t seq[4];

    // 读取四个电机的位置反馈
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t id = i + 1;

        if (!Emm_Feedback[id].valid)
            return;

        pos[i] = Emm_Feedback[id].pos_raw;
        seq[i] = Emm_Feedback[id].seq;

        // 防止使用长时间未更新的位置数据
        if ((HAL_GetTick() - Emm_Feedback[id].tick) > 100)
            return;
    }

    // 等待四个电机都产生新的位置反馈
    for (uint8_t i = 0; i < 4; i++)
    {
        if (seq[i] == motor_last_seq[i])
            return;
    }

    // 第一次运行只记录初始位置，不计算位移
    if (!odometry_init)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            motor_last_pos[i] = pos[i];
            motor_last_seq[i] = seq[i];
        }

        odometry_init = 1;
        return;
    }

    float wheel_ds[4];

    // 计算四个轮子的实际位移
    for (uint8_t i = 0; i < 4; i++)
    {
        int64_t delta = (int64_t)pos[i] - motor_last_pos[i];

       wheel_ds[i] = (float)delta
            * CHASSIS_POS_TO_MM
            * motor_pos_dir[i];

        motor_last_pos[i] = pos[i];
        motor_last_seq[i] = seq[i];
    }

    // 麦克纳姆轮逆运动学
   float dx_body = (wheel_ds[0]
               + wheel_ds[1]
               - wheel_ds[2]
               - wheel_ds[3]) * 0.25f;

   float dy_body = (wheel_ds[0]
               - wheel_ds[1]
               + wheel_ds[2]
               - wheel_ds[3]) * 0.25f;

    // 使用 IMU 航向角，将底盘坐标系位移转换到场地坐标系
    float yaw = IMU_getYaw() * CHASSIS_PI / 180.0f;

    float dx_field = dx_body * cosf(yaw)
                   - dy_body * sinf(yaw);

    float dy_field = dx_body * sinf(yaw)
                   + dy_body * cosf(yaw);



// 调试用
    for (uint8_t i = 0; i < 4; i++)
    {
        debug_wheel_ds[i] = wheel_ds[i];
    }

    debug_dx_body = dx_body;
    debug_dy_body = dy_body;

    debug_dx_field = dx_field;
    debug_dy_field = dy_field;


    // 更新场地坐标
    chassis.Px += dx_field;
    chassis.Py += dy_field;
}

void Chassis_ResetPos(void)
{
    chassis.Px = 0.0f;
    chassis.Py = 0.0f;

    odometry_init = 0;

    Chassis_clear();
}

/* 轮转读四轮 0x36 实时位置：每周期最多补 1 帧，4 个周期覆盖一圈。
   CAN TX FIFO 只有 3 层，而 Chassis_Motor_SetVel 每周期已排入 5 帧；
   这里只在 FIFO 有空位时发，不发就下周期再试，不 osDelay、不阻塞控制环。 */
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
 * 设置底盘速度
 * 单位:mm/s
 * 单位:rad/s
 */
void Chassis_SetVel(float Vx, float Vy, float Vw)
{
    chassis.ctrlMode = VELOCITY_MODE;

    chassis.setVx = Vx;
    chassis.setVy = Vy;

    // 进入速度模式时锁定当前航向
    chassis.setYaw = IMU_getYaw();

    startVx = chassis.nowVx;
    startVy = chassis.nowVy;

    chassis.accel_count = 0;
    chassis.accel_K = 50;

    Chassis_clear();
}
/**
 * 设置底盘位置
 * 单位:mm
 * 单位:rad
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

void Chassis_Drift(float Vx, float Vy, float Yaw)
{
    startVx = chassis.nowVx;
    startVy = chassis.nowVy;

    chassis.setVx = Vx;
    chassis.setVy = Vy;
    chassis.setYaw = Yaw;

    chassis.accel_count = 0;
    chassis.accel_K = 50;

    chassis.ctrlMode = VELOCITY_MODE;

    Chassis_clear();
}
/**
 * @brief 底盘持续旋转漂移
 * @param Vx 场地坐标系 X 方向速度
 * @param Vy 场地坐标系 Y 方向速度
 * @param W  底盘旋转速度，逆时针为正
 */
void Chassis_SpinDrift(float Vx, float Vy, float W)
{
    // 保存当前速度，作为加速起点
    startVx = chassis.nowVx;
    startVy = chassis.nowVy;

    // 设置目标平移速度
    chassis.setVx = Vx;
    chassis.setVy = Vy;

    // 设置持续旋转速度
    chassis.setVw = W;

    // 重置正弦加速过程
    chassis.accel_count = 0;
    chassis.accel_K = 50;

  chassis.ctrlMode = SPIN_DRIFT_MODE;

    Chassis_clear();
}