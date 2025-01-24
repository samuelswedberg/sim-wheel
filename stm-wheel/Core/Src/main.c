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
typedef struct __attribute__((packed)){
	int32_t  tRpm;
	int32_t  tGear;
	int32_t  tSpeedKmh;
	int32_t  tHasDRS;
	int32_t  tDrs;
	int32_t  tPitLim;
	int32_t  tFuel;
	int32_t  tBrakeBias;
} telemetry_packet;

typedef struct __attribute__((__packed__)) {
    uint16_t buttons;       // 10 physical buttons + 2 hall sensor buttons (12 total, packed into 16 bits)
    uint8_t hall_analog_1;  // First hall sensor analog value (0-255)
    uint8_t hall_analog_2;  // Second hall sensor analog value (0-255)
    int16_t encoder_1;      // First encoder value
    int16_t encoder_2;      // Second encoder value
    int16_t encoder_3;      // Third encoder value
} user_input_data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// GLOBAL VARIABLES
telemetry_packet gReceivedTelemetry;

// Oscillation state variables
int value = 0;
int direction = 1; // 1 for increasing, -1 for decreasing
int step = 1;      // Increment/Decrement step
int delay_ms = 25; // Delay between updates

uint32_t lastSendTime = 0;

int gSteering = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_to_hmi(const char *command);
int oscillate_value();
void Flash_Onboard_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay_ms);
void CAN_Transmit();
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // Turn LED off
	  int oscillated_value = oscillate_value();

	  char command[32];
	  snprintf(command, sizeof(command), "rpmbar.val=%d", gSteering);

	  // Send the command to the Nextion display
	  send_to_nextion(command);

	  CAN_Transmit();
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
	  // Wait for the specified delay
//	  HAL_Delay(5);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
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

  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConfig.FilterIdHigh = 0x100 << 5;       // Accept all IDs
  filterConfig.FilterMaskIdHigh = 0x7FF << 5;;   // Accept all IDs
  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;  // Assign to FIFO 1
  filterConfig.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filterConfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

void Flash_Onboard_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay_ms) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Turn LED on
    HAL_Delay(delay_ms);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);   // Turn LED off
    HAL_Delay(delay_ms);
}

/*
 * NEXTION UART FUNCTIONS
 */
void send_to_nextion(const char *command) {
	// Send the command string
	HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

	// Send the termination bytes (0xFF 0xFF 0xFF)
	uint8_t termination_bytes[3] = {0xFF, 0xFF, 0xFF};
	HAL_UART_Transmit(&huart1, termination_bytes, sizeof(termination_bytes), HAL_MAX_DELAY);
}

int oscillate_value() {
    // Update value based on direction
    value += direction * step;

    // Reverse direction at boundaries
    if (value >= 100 || value <= 0) {
        direction *= -1;
    }

    return value;
}

/*
 * CAN BUS FUNCTIONS
 */
//void CAN_Transmit() {
//	CAN_TxHeaderTypeDef txHeader;
//	uint32_t txMailbox;
//	uint8_t data[8] = {100, 101, 102, 103, 104, 105, 106, 107};
//
//	txHeader.StdId = 0x001;
//	txHeader.ExtId = 0;
//	txHeader.IDE = CAN_ID_STD;
//	txHeader.RTR = CAN_RTR_DATA;
//	txHeader.DLC = 8;
//
//	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox);
//	if (status != HAL_OK) {
//		if (status == HAL_ERROR) {
//			printf("HAL_CAN_AddTxMessage failed: HAL_ERROR\n");
//		} else if (status == HAL_BUSY) {
//			printf("HAL_CAN_AddTxMessage failed: HAL_BUSY\n");
//		} else if (status == HAL_TIMEOUT) {
//			printf("HAL_CAN_AddTxMessage failed: HAL_TIMEOUT\n");
//		}
//
//		uint32_t error = HAL_CAN_GetError(&hcan);
//		printf("CAN Error Code: 0x%08lx\n", error);
//	}
//}

void CAN_Transmit() {
	uint32_t currentTime = HAL_GetTick();

	if(currentTime - lastSendTime >= 10) {
		CAN_TxHeaderTypeDef TxHeader;
		uint32_t TxMailbox;

		// Create a telemetry_packet instance and initialize its fields
		user_input_data_t dataToSend;
		dataToSend.buttons = 0x0F0F;         // Example: Buttons pressed
		dataToSend.hall_analog_1 = 100;      // Example: Hall sensor 1 value
		dataToSend.hall_analog_2 = 200;      // Example: Hall sensor 2 value
		dataToSend.encoder_1 = 1000;         // Example: Encoder 1 value
		dataToSend.encoder_2 = -2000;        // Example: Encoder 2 value
		dataToSend.encoder_3 = 5000;         // Example: Encoder 3 value

		uint8_t* rawData = (uint8_t*)&dataToSend;

		// Initialize CAN Header
		TxHeader.StdId = 0x101;           // CAN ID for the message
		TxHeader.ExtId = 0;
		TxHeader.IDE = CAN_ID_STD;        // Use Standard ID
		TxHeader.RTR = CAN_RTR_DATA;      // Data frame
		TxHeader.DLC = 8;                 // Maximum data length for each CAN frame

		uint8_t frameData[8];             // Temporary buffer for each CAN frame

		// Calculate the size of the telemetry_packet struct
		int totalSize = sizeof(user_input_data_t);

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

// CAN receive interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8]; // Max CAN frame size is 8 bytes

    // Receive the message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {
    	 // Check if the message ID matches 0x100
		if (rxHeader.StdId == 0x100) {
			static uint8_t buffer[sizeof(telemetry_packet)];
			static uint8_t offset = 0;

			// Copy received data into buffer
			uint8_t bytesToCopy = (rxHeader.DLC < sizeof(telemetry_packet) - offset) ? rxHeader.DLC : sizeof(telemetry_packet) - offset;
			memcpy(&buffer[offset], rxData, bytesToCopy);
			offset += bytesToCopy;

			// Check if the entire packet has been received
			if (offset >= sizeof(telemetry_packet)) {
				// Copy buffer into the telemetry_packet struct
				memcpy(&gReceivedTelemetry, buffer, sizeof(telemetry_packet));
				offset = 0; // Reset offset for the next packet

				// Process the received telemetry data
				ProcessTelemetryData(&gReceivedTelemetry);
			}
		}
    }
}

void ProcessTelemetryData(const telemetry_packet *data) {
    // Example: Log or handle telemetry values
    gSteering = data->tSpeedKmh;


    // Handle the telemetry data as needed
    // For example, update a display, store in memory, or take action
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
