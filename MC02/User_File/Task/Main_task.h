#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "main.h"

#include "dvc_vofa.h"
#include "dvc_stepmotor.h"
#include "dvc_servo.h"


void Main_Init(void);
void Main_Task(void *argument);

#endif // MAIN_TASK_H