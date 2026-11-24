#ifndef __TOWER_ACTION_H
#define __TOWER_ACTION_H

#include "main.h"

/* ==================== 动作类型 ==================== */
typedef enum
{
    TOWER_ACTION_IDLE = 0,
    TOWER_ACTION_RESET,
    TOWER_ACTION_IDENTIFY,
    TOWER_ACTION_GRAB_GROUND,
    TOWER_ACTION_PUT_GROUND
} Tower_Action_Type_t;


/* ==================== 动作状态机接口 ==================== */

// 初始化动作模块
void Tower_Action_Init(void);

// 启动指定动作
void Tower_Action_Start(Tower_Action_Type_t action);

// 周期执行动作状态机
void Tower_Action_Run(void);

// 查询动作是否完成
uint8_t Tower_Action_IsFinished(void);


/* ==================== 塔吊目标控制接口 ==================== */

// 更新塔吊三个机构的目标位置
void Tower_Target_Update(int16_t yaw,
                         int16_t lift,
                         int16_t telescope);

// 夹爪打开
void Tower_Claw_on(void);

// 夹爪闭合
void Tower_Claw_off(void);


void Tower_Reset_Posture(void);
void Tower_Identify_Posture(void);
/* 地面抓取物料 */
void Tower_Grab_Ground(uint8_t position);

void Tower_Put_Car(uint8_t order);

void Tower_Grab_Tray(uint8_t order);

void Tower_Put_Ground(uint8_t position);

#endif