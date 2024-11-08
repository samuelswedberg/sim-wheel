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
osThreadId defaultTaskHandle;
osThreadId telemetryTaskHandle;
osThreadId heartbeatTaskHandle;
osThreadId SPISendDataTaskHandle;
osThreadId FFBTaskHandle;
osSemaphoreId spiSendSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTelemetryTask(void const * argument);
void StartHeartbeatTask(void const * argument);
void StartSPISend(void const * argument);
void StartFFBTask(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of spiSendSemaphore */
  osSemaphoreDef(spiSendSemaphore);
  spiSendSemaphoreHandle = osSemaphoreCreate(osSemaphore(spiSendSemaphore), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of telemetryTask */
  osThreadDef(telemetryTask, StartTelemetryTask, osPriorityHigh, 0, 128);
  telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);

  /* definition and creation of heartbeatTask */
  osThreadDef(heartbeatTask, StartHeartbeatTask, osPriorityLow, 0, 128);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);

  /* definition and creation of SPISendDataTask */
  osThreadDef(SPISendDataTask, StartSPISend, osPriorityHigh, 0, 128);
  SPISendDataTaskHandle = osThreadCreate(osThread(SPISendDataTask), NULL);

  /* definition and creation of FFBTask */
  osThreadDef(FFBTask, StartFFBTask, osPriorityHigh, 0, 128);
  FFBTaskHandle = osThreadCreate(osThread(FFBTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void const * argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void const * argument)
{
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartSPISend */
/**
* @brief Function implementing the SPISendDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPISend */
void StartSPISend(void const * argument)
{
  /* USER CODE BEGIN StartSPISend */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSPISend */
}

/* USER CODE BEGIN Header_StartFFBTask */
/**
* @brief Function implementing the FFBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFFBTask */
void StartFFBTask(void const * argument)
{
  /* USER CODE BEGIN StartFFBTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartFFBTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
