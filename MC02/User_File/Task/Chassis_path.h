#ifndef CHASSIS_PATH_H
#define CHASSIS_PATH_H

#include "main.h"

#define PATH_MAX_POINTS  20

typedef struct
{
    float x;       // 场地X坐标，mm
    float y;       // 场地Y坐标，mm
    float yaw;     // 目标航向角，degree

    float speed;   // 路径点期望速度，mm/s
} PathPoint_t;

typedef struct
{
    PathPoint_t point[PATH_MAX_POINTS];

    uint8_t point_count;
    uint8_t current_segment;

    float total_time;
    float current_time;

    uint8_t finished;

} ChassisPath_t;

void Chassis_Path_Init(void);
void Chassis_Path_Start(void);
void Chassis_Path_Update(void);

#endif