#include "Tower_action.h"
#include "Tower_control.h"


/**
 * @brief 塔吊目标位置更新
 * @param yaw        云台目标角度, 相对于舵机设定中位（0°）
 * @param lift       升降目标位置，mm
 * @param telescope  伸缩目标位置，mm
 */
void Tower_Target_Update(int16_t yaw,
                         int16_t lift,
                         int16_t telescope)
{
    Tower_Control(yaw,
                  lift,
                  1000,
                  telescope,
                  1000);
}



void Tower_Claw_on(void)
{
    Tower_SetClaw_Angle(0);  // 打开
}

void Tower_Claw_off(void)
{
    Tower_SetClaw_Angle(90);  // 闭关
}


/* 当前动作和动作状态 */
static Tower_Action_Type_t current_action = TOWER_ACTION_IDLE;

typedef enum
{
    ACTION_STEP_IDLE = 0,
    ACTION_STEP_MOVE,
    ACTION_STEP_FINISHED
} Tower_Action_Step_t;

static Tower_Action_Step_t current_step = ACTION_STEP_IDLE;

/* 初始化动作模块 */
void Tower_Action_Init(void)
{
    current_action = TOWER_ACTION_IDLE;
    current_step = ACTION_STEP_IDLE;
}

/* 启动指定动作 */
void Tower_Action_Start(Tower_Action_Type_t action)
{
    if (current_step != ACTION_STEP_IDLE &&
        current_step != ACTION_STEP_FINISHED)
    {
        return;   // 当前动作未结束，不接受新动作
    }

    if (action == TOWER_ACTION_IDLE)
    {
        current_action = TOWER_ACTION_IDLE;
        current_step = ACTION_STEP_IDLE;
        return;
    }

    current_action = action;
    current_step = ACTION_STEP_IDLE;
}

/* 查询动作是否完成 */
uint8_t Tower_Action_IsFinished(void)
{
    return (current_step == ACTION_STEP_FINISHED);
}

/* 周期执行动作 */
void Tower_Action_Run(void)
{
    if (current_step != ACTION_STEP_IDLE)
        return;

    switch (current_action)
    {
        case TOWER_ACTION_RESET:
            /* 复位姿态 */
            Tower_Target_Update(0, 50, 15);
            current_step = ACTION_STEP_FINISHED;
            break;

        case TOWER_ACTION_IDENTIFY:
            /* 识别姿态 */
            Tower_Target_Update(0, 100, 80);
            current_step = ACTION_STEP_FINISHED;
            break;

        case TOWER_ACTION_GRAB_GROUND:
            /* 地面抓取姿态 */
            Tower_Target_Update(30, 30, 80);
            current_step = ACTION_STEP_FINISHED;
            break;

        case TOWER_ACTION_PUT_GROUND:
            /* 地面放置姿态：等待确定实际位置 */
            break;

        default:
            break;
    }
}