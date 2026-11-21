#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include "main.h"

// 轮子参数
#define CHASSIS_WHEEL_RADIUS       40.0f
#define CHASSIS_POS_COUNT_PER_REV  65536.0f
#define CHASSIS_PI                 3.1415926f

#define CHASSIS_POS_TO_MM \
    (2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS / CHASSIS_POS_COUNT_PER_REV)

// 里程计数据
typedef struct
{
    float Px;
    float Py;

    float dx_body;
    float dy_body;

    float dx_field;
    float dy_field;

    float wheel_ds[4];

    uint8_t initialized;
} Chassis_Odometry_t;

// 初始化
void Chassis_Odometry_Init(void);

// 复位位置
void Chassis_Odometry_Reset(void);

// 更新里程计
void Chassis_Odometry_Update(void);

// 获取位置
float Chassis_Odometry_GetX(void);
float Chassis_Odometry_GetY(void);

#endif
