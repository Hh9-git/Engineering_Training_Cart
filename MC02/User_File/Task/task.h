#ifndef TASK_H
#define TASK_H

#include "main.h"
#include "usart.h"

#include "bsp_can.h"

#include "dvc_vofa.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"
#include "drv_usart.h"
#include "drv_can.h"
#include "drv_pwm.h"

void Task_Init(void);
void Task_Loop(void);

#endif // TASK_H