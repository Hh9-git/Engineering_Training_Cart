/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Tower */
osThreadId_t TowerHandle;
const osThreadAttr_t Tower_attributes = {
  .name = "Tower",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Main */
osThreadId_t MainHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Debug */
osThreadId_t DebugHandle;
const osThreadAttr_t Debug_attributes = {
  .name = "Debug",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Lcd */
osThreadId_t LcdHandle;
const osThreadAttr_t Lcd_attributes = {
  .name = "Lcd",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Chassis_Task(void *argument);
void Tower_Task(void *argument);
void Main_Task(void *argument);
void Debug_Task(void *argument);
void Lcd_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

   /* 没有调试器，所以用灯珠报告：常亮纯红 = 任务栈溢出。
    * 注意不要在这里返回 —— 溢出后栈已经不可信了。 */
   (void)xTask;
   (void)pcTaskName;

   WS2812_Ctrl(255, 0, 0);
   for (;;) { }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(Chassis_Task, NULL, &Chassis_attributes);

  /* creation of Tower */
  TowerHandle = osThreadNew(Tower_Task, NULL, &Tower_attributes);

  /* creation of Main */
  MainHandle = osThreadNew(Main_Task, NULL, &Main_attributes);

  /* creation of Debug */
  DebugHandle = osThreadNew(Debug_Task, NULL, &Debug_attributes);

  /* creation of Lcd */
  LcdHandle = osThreadNew(Lcd_Task, NULL, &Lcd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 恢复中断使能。上面每一次 osThreadNew 都会经由 pvPortMalloc /
   * prvAddNewTaskToReadyList 进入临界区，而调度器启动前 port.c 里
   * uxCriticalNesting 的初值是 0xaaaaaaaa（"未启动"哨兵值），
   * vPortExitCritical() 退出时永远回不到 0，portENABLE_INTERRUPTS()
   * 不会执行 —— BASEPRI 被永久抬到 0x50，屏蔽所有优先级 >= 5 的中断，
   * 而 TIM23(HAL 时基) 和 SysTick 都是优先级 15。
   * 后果就是 HAL_Delay 永久卡死、FreeRTOS 心跳不走、系统整体冻结。 */
  __set_BASEPRI(0U);

  /* 品红常亮 = 有任务返回 NULL（堆不够）。osThreadNew 失败时既不报错也不跑，
   * 只能这样看出来。 */
  if ((defaultTaskHandle == NULL) || (ChassisHandle == NULL) || (TowerHandle == NULL) ||
      (MainHandle == NULL) || (DebugHandle == NULL))
  {
    WS2812_Ctrl(255, 0, 255);
    for (;;) { }
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* 必须在这里再清一次 BASEPRI：vTaskStartScheduler() 为 idle/timer 任务做
   * pvPortMalloc 时又把它抬起来了，而 xPortStartScheduler() 只把
   * uxCriticalNesting 归零，并不会放下 BASEPRI。
   * BASEPRI 是内核级全局寄存器，清一次所有任务和中断就都恢复。
   *
   * 注意：本任务是调度器启动后第一个运行的（优先级 24，最高），所以它是
   * 唯一能保证执行到这里的任务。如果以后改了任务优先级，要保证仍然有
   * 某个最早运行的任务执行这句清理，否则整个系统会再次冻结。 */
  __set_BASEPRI(0U);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void *argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Tower_Task */
/**
* @brief Function implementing the Tower thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tower_Task */
__weak void Tower_Task(void *argument)
{
  /* USER CODE BEGIN Tower_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Tower_Task */
}

/* USER CODE BEGIN Header_Main_Task */
/**
* @brief Function implementing the Main thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Main_Task */
__weak void Main_Task(void *argument)
{
  /* USER CODE BEGIN Main_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Main_Task */
}

/* USER CODE BEGIN Header_Debug_Task */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug_Task */
__weak void Debug_Task(void *argument)
{
  /* USER CODE BEGIN Debug_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_Task */
}

/* USER CODE BEGIN Header_Lcd_Task */
/**
* @brief Function implementing the Lcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lcd_Task */
__weak void Lcd_Task(void *argument)
{
  /* USER CODE BEGIN Lcd_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Lcd_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 栈溢出钩子已挪到文件上方的 USER CODE BEGIN 4 段 —— 那是 CubeMX 为它准备的
 * 位置，重新生成代码时会保留。这里不要再定义一遍，否则会重复定义。 */

/* USER CODE END Application */

