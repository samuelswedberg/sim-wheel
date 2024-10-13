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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <cJSON.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t command_data[BUFFER_SIZE];  // Buffer to hold a copy of the received command
volatile uint8_t command_flag = 0;

// Telemetry data
volatile int tGear = 0;
volatile int tRpm = 0;
volatile int tSpeedKmh = 0;
volatile int tHasDRS = 0;
volatile int tDrs = 0;
volatile int tPitLim = 0;
volatile int tFuel = 0;
volatile int tBrakeBias = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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

		// Check if items were found and extract values
		if (cJSON_IsNumber(rpm)) { tRpm = rpm->valueint; }
		if (cJSON_IsNumber(gear)) { tGear = gear->valueint; }
		if (cJSON_IsNumber(speedKmh)) { tSpeedKmh = speedKmh->valueint; }
		if (cJSON_IsNumber(hasDRS)) { tHasDRS = hasDRS->valueint; }
		if (cJSON_IsNumber(drs)) { tDrs = drs->valueint; }
		if (cJSON_IsNumber(pitLim)) { tPitLim = pitLim->valueint; }
		if (cJSON_IsNumber(fuel)) { tFuel = fuel->valueint; }
		if (cJSON_IsNumber(brakeBias)) { tBrakeBias = brakeBias->valueint; }
		}
		// Cleanup
		cJSON_Delete(json_data);
		// Clear the buffer for the next message
		memset(command_data, 0, BUFFER_SIZE);
		// Re-enable UART reception
		HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(command_flag)
	  {
		  process_command(command_data);
		  command_flag = 0;
		  HAL_Delay(50); // ! Increase this is value update is slower due to more telemetry data params
		  if(tRpm >= 7000)
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  }
		  else
		  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		  }
		  // Re-enable UART reception
		  HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
	  }
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
  huart2.Init.BaudRate = 921600;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Process the received data (rx_buffer)
    	memcpy(command_data, rx_buffer, sizeof(rx_buffer));
    	command_flag = 1; // Reset flag
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
        	//send_response("UART Framing Error");
        }

        // Handle Parity Error (PE)
        if (error_code & HAL_UART_ERROR_PE) {
            // Parity errors may indicate data corruption or mismatch in settings
        	//send_response("UART Parity Error");
        }

        // Handle Noise Error (NE)
        if (error_code & HAL_UART_ERROR_NE) {
            // Noise errors are usually transient but worth logging
        	//send_response("UART Noise Error");
        }

        // Recovery: Restart UART reception after clearing the error flags
        HAL_UART_Receive_IT(huart, rx_buffer, sizeof(rx_buffer));
    }
}

/* USER CODE END 4 */

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
