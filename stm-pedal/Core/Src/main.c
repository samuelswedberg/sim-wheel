/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct __attribute__((packed, aligned(1))) {
    int16_t encoder_1;      // First encoder value
    int16_t encoder_2;      // Second encoder value
    int16_t encoder_3;      // Third encoder value
} pedal_data_t;

pedal_data_t pedal_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
uint32_t lastSendTime = 0;
uint32_t adc_values[3];  // Store ADC readings for PA0, PA1, PA2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Flash_Onboard_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay_ms);
void CAN_Transmit();
void Read_ADC_Value();
void Read_Potentiometers();
uint8_t map_hall_sensor(uint16_t adc_value);
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
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // Turn LED off
	  Read_Potentiometers();
	 CAN_Transmit();
	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
//	 HAL_Delay(50);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef filterConfig;

  filterConfig.FilterBank = 0;                  // Use filter bank 0 (adjust if needed)
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Mask mode
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32-bit scale
  filterConfig.FilterIdHigh = 0xFFFF;           // Invalid ID
  filterConfig.FilterIdLow = 0xFFFF;            // Invalid ID
  filterConfig.FilterMaskIdHigh = 0xFFFF;       // Mask blocks all
  filterConfig.FilterMaskIdLow = 0xFFFF;        // Mask blocks all
  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; // Assign to FIFO0 (or FIFO1 if used)
  filterConfig.FilterActivation = ENABLE;       // Activate the filter

  HAL_CAN_ConfigFilter(&hcan, &filterConfig);

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * CAN BUS FUNCTIONS
 */

void CAN_Transmit() {
	uint32_t currentTime = HAL_GetTick();

	if(currentTime - lastSendTime >= 37) {
		CAN_TxHeaderTypeDef TxHeader;
		uint32_t TxMailbox;

		// Create a telemetry_packet instance and initialize its fields
		pedal_data_t dataToSend = pedal_data;
//		dataToSend.encoder_1 = 1234;         // Example: Encoder 1 value
//		dataToSend.encoder_2 = -2234;        // Example: Encoder 2 value
//		dataToSend.encoder_3 = 5234;         // Example: Encoder 3 value

		uint8_t* rawData = (uint8_t*)&dataToSend;

		// Initialize CAN Header
		TxHeader.StdId = 0x102;           // CAN ID for the message
		TxHeader.ExtId = 0;
		TxHeader.IDE = CAN_ID_STD;        // Use Standard ID
		TxHeader.RTR = CAN_RTR_DATA;      // Data frame
		TxHeader.DLC = 8;                 // Maximum data length for each CAN frame

		uint8_t frameData[8];             // Temporary buffer for each CAN frame

		// Calculate the size of the telemetry_packet struct
		int totalSize = sizeof(pedal_data_t);

		// Split the telemetry_packet into CAN frames
		for (int i = 0; i < totalSize; i += 8) {
			// Calculate the size of the current chunk (for the last frame)
			int chunkSize = (totalSize - i >= 8) ? 8 : (totalSize - i);

			// Copy the next chunk of data into the frame buffer
			memcpy(frameData, &rawData[i], chunkSize);

			// Adjust DLC for the last frame
			TxHeader.DLC = chunkSize;

			HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, frameData, &TxMailbox);
			if (status != HAL_OK) {
				// Inspect the error
				if (status == HAL_ERROR) {
					printf("HAL_CAN_AddTxMessage failed: HAL_ERROR\n");
				} else if (status == HAL_BUSY) {
					printf("HAL_CAN_AddTxMessage failed: HAL_BUSY\n");
				} else if (status == HAL_TIMEOUT) {
					printf("HAL_CAN_AddTxMessage failed: HAL_TIMEOUT\n");
				}

				// Optionally log the state of CAN error counters
				uint32_t error = HAL_CAN_GetError(&hcan);
				printf("CAN Error Code: 0x%08lx\n", error); // Only if you decide to stop execution
			}
			lastSendTime = currentTime;  // Update last transmission time
			HAL_Delay(1);
		}
	}
}

void Read_ADC_Value() {
    HAL_ADC_Start(&hadc1);

    for (int i = 0; i < 3; i++) {
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adc_values[i] = HAL_ADC_GetValue(&hadc1);
    }

    HAL_ADC_Stop(&hadc1);
}

void Read_Potentiometers() {
	Read_ADC_Value();
	pedal_data.encoder_1 = map_hall_sensor(adc_values[0]);  // Read PA0 (ADC1_IN0)
	pedal_data.encoder_2 = map_hall_sensor(adc_values[1]);  // Read PA1 (ADC1_IN1)
	pedal_data.encoder_3 = map_hall_sensor(adc_values[2]);  // Read PA2 (ADC1_IN2)
}

uint8_t map_hall_sensor(uint16_t adc_value) {
    if (adc_value > 4000) adc_value = 4000;  // Ensure it stays within range
    return (uint8_t)((adc_value * 255) / 4000);
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
