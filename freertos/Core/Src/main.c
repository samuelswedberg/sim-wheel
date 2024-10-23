/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cJSON.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((__packed__)) {
	int32_t  tRpm;
	int32_t  tGear;
	int32_t  tSpeedKmh;
	int32_t  tHasDRS;
	int32_t  tDrs;
	int32_t  tPitLim;
	int32_t  tFuel;
	int32_t  tBrakeBias;
	int32_t tForceFB;
} telemetry_packet;

telemetry_packet telemetry_data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
#define NUM_INTS 8
#define FLOAT_SIZE sizeof(float)
#define DATA_SIZE (NUM_INTS * sizeof(int) + FLOAT_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
TaskHandle_t spiTaskHandle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId telemetryTaskHandle;
osThreadId heartbeatTaskHandle;
osThreadId SPISendDataTaskHandle;
/* USER CODE BEGIN PV */
uint8_t rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t tx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t gCommandData[BUFFER_SIZE];  // Buffer to hold a copy of the received command

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTelemetryTask(void const * argument);
void StartHeartbeatTask(void const * argument);
void StartSPISend(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_command(char* cmd) {
	cJSON *json_data = cJSON_Parse(cmd);
	if (json_data != NULL) {
		// Extract data from the JSON object
		cJSON *rpm = cJSON_GetObjectItem(json_data, "rpm");
		cJSON *gear = cJSON_GetObjectItem(json_data, "gear");
		cJSON *speedKmh = cJSON_GetObjectItem(json_data, "speedKmh");
		cJSON *hasDRS = cJSON_GetObjectItem(json_data, "hasDRS");
		cJSON *drs = cJSON_GetObjectItem(json_data, "drs");
		cJSON *pitLim = cJSON_GetObjectItem(json_data, "pitLim");
		cJSON *fuel = cJSON_GetObjectItem(json_data, "fuel");
		cJSON *brakeBias = cJSON_GetObjectItem(json_data, "brakeBias");
		cJSON *forceFB = cJSON_GetObjectItem(json_data, "forceFB");

		// Check if items were found and extract values
		if (cJSON_IsNumber(rpm)) { telemetry_data.tRpm = rpm->valueint; }
		if (cJSON_IsNumber(gear)) { telemetry_data.tGear = gear->valueint; }
		if (cJSON_IsNumber(speedKmh)) { telemetry_data.tSpeedKmh = speedKmh->valueint; }
		if (cJSON_IsNumber(hasDRS)) { telemetry_data.tHasDRS = hasDRS->valueint; }
		if (cJSON_IsNumber(drs)) { telemetry_data.tDrs = drs->valueint; }
		if (cJSON_IsNumber(pitLim)) { telemetry_data.tPitLim = pitLim->valueint; }
		if (cJSON_IsNumber(fuel)) { telemetry_data.tFuel = fuel->valueint; }
		if (cJSON_IsNumber(brakeBias)) { telemetry_data.tBrakeBias = brakeBias->valueint; }
		if (cJSON_IsNumber(forceFB)) { telemetry_data.tForceFB = (float)forceFB->valuedouble; }
		}
		// Cleanup
		cJSON_Delete(json_data);
		// Clear the buffer for the next message
		memset(gCommandData, 0, BUFFER_SIZE);
}

void send_response(const char* str) {
    if (str == NULL) {
        return; // Handle null pointer case if necessary
    }

    // Calculate the length of the string
    uint16_t len = strlen(str);

    // Transmit the string using HAL_UART_Transmit
    HAL_UART_Transmit(&huart2, (uint8_t*)str, len, HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  telemetry_data.tRpm = 0;
  telemetry_data.tRpm = 0;
  telemetry_data.tSpeedKmh = 0;
  telemetry_data.tHasDRS = 0;
  telemetry_data.tDrs = 0;
  telemetry_data.tPitLim = 0;
  telemetry_data.tFuel = 0;
  telemetry_data.tBrakeBias = 0;
  telemetry_data.tForceFB = 0;
  memset(&telemetry_data, 0, sizeof(telemetry_packet)); // Zero-initialize
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of telemetryTask */
  osThreadDef(telemetryTask, StartTelemetryTask, osPriorityRealtime, 0, 128);
  telemetryTaskHandle = osThreadCreate(osThread(telemetryTask), NULL);

  /* definition and creation of heartbeatTask */
  osThreadDef(heartbeatTask, StartHeartbeatTask, osPriorityLow, 0, 128);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);

  /* definition and creation of SPISendDataTask */
  osThreadDef(SPISendDataTask, StartSPISend, osPriorityRealtime, 0, 128);
  SPISendDataTaskHandle = osThreadCreate(osThread(SPISendDataTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Task creation
  // Start scheduler
  vTaskStartScheduler();
  send_response("STM Started");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Process the received data (rx_buffer)
        memcpy(gCommandData, rx_buffer, sizeof(rx_buffer));
        // Notify the telemetry task to process the command
        osSignalSet(telemetryTaskHandle, 0x01);  // Set signal for telemetry task
        // Clear the buffer for the next message
        memset(rx_buffer, 0, BUFFER_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uint32_t error_code = HAL_UART_GetError(huart);

    // Identify which UART instance is causing the error (USART2 in this case)
    if (huart->Instance == USART2) {

        // Handle Overrun Error (ORE)
        if (error_code & HAL_UART_ERROR_ORE) {
            __HAL_UART_CLEAR_OREFLAG(huart);  // Clear overrun error flag
            // Optionally log or handle the error
            //send_response("UART Overrun Error");
        }

        // Handle Framing Error (FE)
        if (error_code & HAL_UART_ERROR_FE) {
            // Clear framing error flag automatically by reading the status register
        	send_response("UART Framing Error");
        }

        // Handle Parity Error (PE)
        if (error_code & HAL_UART_ERROR_PE) {
            // Parity errors may indicate data corruption or mismatch in settings
        	send_response("UART Parity Error");
        }

        // Handle Noise Error (NE)
        if (error_code & HAL_UART_ERROR_NE) {
            // Noise errors are usually transient but worth logging
        	send_response("UART Noise Error");
        }

        // Recovery: Restart UART reception after clearing the error flags
        HAL_UART_Receive_IT(huart, rx_buffer, sizeof(rx_buffer));
    }
}

void packTelemetryPacket(telemetry_packet *packet, uint8_t *buffer) {
    // Ensure buffer is large enough
    if (buffer == NULL) return;  // Add appropriate size check if needed

    size_t offset = 0;

    // Helper macro to pack data in little-endian
    #define PACK_INT32(value) \
        memcpy(buffer + offset, &value, sizeof(value)); \
        offset += sizeof(value);

    #define PACK_FLOAT(value) \
        memcpy(buffer + offset, &value, sizeof(value)); \
        offset += sizeof(value);

    // Pack the structure fields into the buffer
    PACK_INT32(packet->tRpm);
    PACK_INT32(packet->tGear);
    PACK_INT32(packet->tSpeedKmh);
    PACK_INT32(packet->tHasDRS);
    PACK_INT32(packet->tDrs);
    PACK_INT32(packet->tPitLim);
    PACK_INT32(packet->tFuel);
    PACK_INT32(packet->tBrakeBias);
    PACK_INT32(packet->tForceFB);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
	// Wait for notification from UART callback
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	// Process the command received via UART
	process_command(gCommandData);

	// Introduce a delay if necessary
	vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
	// Re-enable UART reception
	HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));

	osSignalSet(SPISendDataTaskHandle, 0x01);  // Set signal for telemetry task
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
	// Perform actions based on telemetry data
	if (telemetry_data.tRpm >= 7000) {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	} else {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
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
	HAL_StatusTypeDef status;
	uint8_t buffer[sizeof(telemetry_packet)];
  for(;;)
  {
  // Wait for notification from UART callback
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	packTelemetryPacket(&telemetry_data, buffer);

	// Chip Select pin low to start transmission
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Adjust GPIO port and pin
	//HAL_Delay(1); // 1 millisecond delay (or adjust as needed)
	// Transmit data
	status = HAL_SPI_Transmit(&hspi2, buffer, sizeof(telemetry_packet), HAL_MAX_DELAY);

	// Chip Select pin high to end transmission
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	// Check for errors
	if (status != HAL_OK) {
		send_response("SPI Transmission Error");
	}

	// Delay for a while to avoid flooding the Pico
	//vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust the delay as needed
  }
  /* USER CODE END StartSPISend */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
