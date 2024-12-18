/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
osThreadId LED_TaskHandle;
osThreadId Motor_TaskHandle;
osThreadId INS_TASKHandle;
osThreadId Chassis_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LEDTask(void const * argument);
void MotorTask(void const * argument);
void INS_Task(void const * argument);
void Chassis_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, LEDTask, osPriorityNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of Motor_Task */
  osThreadDef(Motor_Task, MotorTask, osPriorityNormal, 0, 128);
  Motor_TaskHandle = osThreadCreate(osThread(Motor_Task), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 2560);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of Chassis_TASK */
  osThreadDef(Chassis_TASK, Chassis_Task, osPriorityHigh, 0, 128);
  Chassis_TASKHandle = osThreadCreate(osThread(Chassis_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LEDTask */
/**
  * @brief  Function implementing the LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDTask */
__weak void LEDTask(void const * argument)
{
  /* USER CODE BEGIN LEDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LEDTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask */
__weak void MotorTask(void const * argument)
{
  /* USER CODE BEGIN MotorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
