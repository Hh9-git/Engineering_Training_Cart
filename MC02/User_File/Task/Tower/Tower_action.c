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
                  3000,
                  telescope,
                  3000);
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
            Tower_Reset_Posture();
            current_step = ACTION_STEP_FINISHED;
            break;

        case TOWER_ACTION_IDENTIFY:
            /* 识别姿态 */
            Tower_Identify_Posture();
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





/**
 * @brief 塔吊复位姿态
 * @note 夹爪打开，云台、升降和伸缩机构回到初始位置
 */
 void Tower_Reset_Posture(void)
{
    /* 夹爪打开 */
    Tower_Claw_on();

    /* 塔吊机构回到复位位置 */
    Tower_Target_Update(0, 140, 5);
    osDelay(1000);  // 等待机构移动到位
}




/**
 * @brief 塔吊识别姿态
 * @note 云台回到中位，升降和伸缩机构移动到识别位置
 */
void Tower_Identify_Posture(void)
{
    /* 夹爪打开 */
    Tower_Claw_on();

    /* 塔吊机构移动到识别位置 */
    Tower_Target_Update(0, 130, 5);
}



/**
 * @brief 地面抓取物料
 * @param position 1左侧，2中间，3右侧
 */
void Tower_Grab_Ground(uint8_t position)
{
    int16_t yaw;
    int16_t telescope;

    switch (position)
    {
        case 1:  // 左侧：物料3
            yaw = 45;
            telescope = 60;
            break;

        case 2:  // 中间：物料2
            yaw = 0;
            telescope = 0;
            break;

        case 3:  // 右侧：物料1
            yaw = -45;
            telescope = 60;
            break;

        default:
            return;
    }

    /* 1. 打开夹爪 */
    Tower_Claw_on();
    osDelay(500);

    /* 2. 移动到物料上方
     *    云台 100°/s，最坏情况从 -125° 转到 +45°（170°）要 1.7s */
    Tower_Target_Update(yaw, 140, telescope);
    osDelay(1800);

    /* 3. 下降到抓取高度 */
    Tower_Target_Update(yaw, 35, telescope);
    osDelay(1500);

    /* 4. 闭合夹爪 */
    Tower_Claw_off();
    osDelay(1000);

     /*5上升到转移高度 */
    Tower_Target_Update(yaw, 140, telescope);
    osDelay(100);


}




/**
 * @brief 将物料放置到地面
 * @param position 1左侧，2中间，3右侧
 */
void Tower_Put_Ground(uint8_t position)
{
    int16_t yaw;
    int16_t telescope;

    /* 1. 根据位置选择放置坐标 */
    switch (position)
    {
        case 1:  // 左侧
            yaw = 45;
            telescope = 70;
            break;

        case 2:  // 中间
            yaw = 0;
            telescope = 0;
            break;

        case 3:  // 右侧
            yaw = -45;
            telescope = 70;
            break;

        default:
            return;
    }

    /* 2. 移动到放置位置上方（云台 100°/s，满行程 2.7s） */
    Tower_Target_Update(yaw, 140, telescope);
    osDelay(1800);

    /* 3. 下降到放置高度 */
    Tower_Target_Update(yaw, 35, telescope);
    osDelay(1000);

    /* 4. 打开夹爪，释放物料 */
    Tower_Claw_on();
    osDelay(500);

    /* 5. 抬升到安全高度 */
    Tower_Target_Update(yaw, 140, telescope);
    osDelay(1000);


}

/**
 * @brief 将物料放置到车上托盘
 * @param order 1、2、3号放置位置
 */
void Tower_Put_Car(uint8_t order)
{
    if (order < 1 || order > 3)
        return;

    switch (order)
    {
        case 1:
            Tower_SetTray_Angle(135);
            break;
        case 2: 
            Tower_SetTray_Angle(0);
            break;
        case 3:
            Tower_SetTray_Angle(-110);
            break;
        default:
            break;
    }    
    osDelay(1000);


   /* 5. 抬升到安全高度（云台从上一位置转到 -125° 最多 170°） */
    Tower_Target_Update(-125, 140, 45);
    osDelay(1800);


    /* 3. 打开夹爪，释放物料 */
    Tower_Claw_on();
    osDelay(1000);

}


/**
 * @brief 从车上托盘抓取物料
 * @param order 1、2、3号抓取位置
 */
void Tower_Grab_Tray(uint8_t order)
{
    if (order < 1 || order > 3)
        return;
    
        /* 2. 打开夹爪 */
    Tower_Claw_on();
    osDelay(100);

        osDelay(1000);
    /* 3. 移动到转盘物料抓取位置 */
    Tower_Target_Update(-125, 140, 40);  // 暂定位置
    //  /* 5. 抬升到安全高度 */
    // Tower_Target_Update(-125, 140, 60);
    osDelay(1000);

   
    /* 1. 转盘转到对应物料位置 */
    switch (order)
    {
        case 1:
            Tower_SetTray_Angle(135);
            break;

        case 2:
            Tower_SetTray_Angle(0);
            break;

        case 3:
            Tower_SetTray_Angle(-110);
            break;

        default:
            return;
    }

    osDelay(1000);


      /* 3. 移动到转盘物料抓取位置 */
    Tower_Target_Update(-125, 110, 40);  // 暂定位置
    osDelay(800);

    /* 4. 闭合夹爪，抓取物料 */
    Tower_Claw_off();
    osDelay(1000);

    /* 5. 抬升到安全高度 */
    Tower_Target_Update(-125, 140, 50);
    osDelay(1000);
}


//摄像头画面中的目标中心点（320, 230）对应的坐标
#define center_x 320
#define center_y 230


