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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed, aligned(1))){
	int32_t  tRpm;
	int32_t  tGear;
	int32_t  tSpeedKmh;
	int32_t  tHasDRS;
	int32_t  tDrs;
	int32_t  tPitLim;
	int32_t  tFuel;
	int32_t  tBrakeBias;
	int32_t  tMaxRpm;
	float   tForceFB;
} telemetry_packet;

telemetry_packet telemetry_data;

typedef struct __attribute__((packed, aligned(1))) {
    uint16_t buttons;       // 10 physical buttons + 2 hall sensor buttons (12 total, packed into 16 bits)
    uint8_t hall_analog_1;  // First hall sensor analog value (0-255)
    uint8_t hall_analog_2;  // Second hall sensor analog value (0-255)
    int16_t encoder_1;      // First encoder value
    int16_t encoder_2;      // Second encoder value
    int16_t encoder_3;      // Third encoder value
} user_input_data_t;

user_input_data_t user_input_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_1_PIN  GPIO_PIN_4 // PA4
#define BUTTON_2_PIN  GPIO_PIN_5 // PA5
#define BUTTON_3_PIN  GPIO_PIN_6 // PA6
#define BUTTON_4_PIN  GPIO_PIN_7 // PA7
#define BUTTON_5_PIN  GPIO_PIN_0 // PB0
#define BUTTON_6_PIN  GPIO_PIN_1 // PB1
#define BUTTON_7_PIN  GPIO_PIN_10 // PB10
#define BUTTON_8_PIN  GPIO_PIN_11 // PB11
#define BUTTON_9_PIN  GPIO_PIN_9 // PB9
#define BUTTON_10_PIN GPIO_PIN_8 // PB8

#define HALL_BUTTON_1_PIN GPIO_PIN_9 // PA0
#define HALL_BUTTON_2_PIN GPIO_PIN_8 // PA1

#define HALL_ANALOG_1_PIN GPIO_PIN_15 // PA2
#define HALL_ANALOG_2_PIN GPIO_PIN_14 // PA3

#define L_ENC_PIN_A GPIO_PIN_7
#define L_ENC_PIN_B GPIO_PIN_6
#define C_ENC_PIN_A GPIO_PIN_4
#define C_ENC_PIN_B GPIO_PIN_5
#define R_ENC_PIN_A GPIO_PIN_3
#define R_ENC_PIN_B GPIO_PIN_15

#define NEOPIXEL_PIN GPIO_PIN_8


#define ADC_CHANNEL_COUNT 4
#define ADC_REST 2000
#define ADC_MAX 2800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// Oscillation state variables
int value = 0;
int direction = 1; // 1 for increasing, -1 for decreasing
int step = 1;      // Increment/Decrement step
int delay_ms = 25; // Delay between updates

uint32_t lastSendTime = 0;
uint16_t adc_values[ADC_CHANNEL_COUNT];

volatile uint8_t adc_data_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int oscillate_value();
void Flash_Onboard_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay_ms);
void CAN_Transmit();
void updateTelemetry();
void updateUserInput();
void send_to_nextion(const char *var_name, int value);
void send__char_to_nextion(const char *var_name, char *value);
void read_hall_sensors();
int32_t map_rpmbar(int32_t input, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
const char* map_gear(int value);
char* int_to_string(int value);
void Start_ADC_DMA();
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
	telemetry_data.tRpm = 0;
	telemetry_data.tRpm = 0;
	telemetry_data.tSpeedKmh = 0;
	telemetry_data.tHasDRS = 0;
	telemetry_data.tDrs = 0;
	telemetry_data.tPitLim = 0;
	telemetry_data.tFuel = 0;
	telemetry_data.tBrakeBias = 0;

	Start_ADC_DMA();
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // Turn LED off

	  updateTelemetry();
	  updateUserInput();

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
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void updateTelemetry() {
//	int mappedRpm = map_value(telemetry_data.tRpm, 0, telemetry_data.tMaxRpm, 0, 100);
	char *mappedRpm = int_to_string(telemetry_data.tRpm);
	char *mappedGear = map_gear(telemetry_data.tGear);
	char *mappedSpeed = int_to_string(telemetry_data.tSpeedKmh);
	char *mappedHasDrs = int_to_string(telemetry_data.tHasDRS);
	char *mappedPitLim = int_to_string(telemetry_data.tPitLim);
	char *mappedFuel = int_to_string(telemetry_data.tFuel);
	char *mappedBrakeBias = int_to_string(telemetry_data.tBrakeBias);

//	send_int_to_nextion("rpmbar", mappedRpm);
	send__char_to_nextion("rpm", mappedRpm);
	send__char_to_nextion("gear", mappedGear);
	send__char_to_nextion("speed", mappedSpeed);
	//send__char_to_nextion("mappedHasDrs", mappedHasDrs);
	//send_to_nextion("pitlim", telemetry_data.tPitLim);
	send__char_to_nextion("fuel", mappedFuel);
	//send_to_nextion("gear", telemetry_data.tBrakeBias);

	if(mappedRpm) {
		free(mappedRpm);
	}
	// dont do gear since thats not int to string
	if(mappedSpeed) {
		free(mappedSpeed);
	}
	if(mappedHasDrs) {
		free(mappedHasDrs);
	}
	if(mappedPitLim) {
		free(mappedPitLim);
	}
	if(mappedFuel) {
		free(mappedFuel);
	}
	if(mappedBrakeBias) {
		free(mappedBrakeBias);
	}

}

void send_int_to_nextion(const char *var_name, int value) {
	char command[32];
	snprintf(command, sizeof(command), "%s.val=%d", var_name, value);

	HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

	uint8_t termination_bytes[3] = {0xFF, 0xFF, 0xFF};
	HAL_UART_Transmit(&huart1, termination_bytes, sizeof(termination_bytes), HAL_MAX_DELAY);
}

void send__char_to_nextion(const char *var_name, char *value) {
	char command[32];
	snprintf(command, sizeof(command), "%s.txt=\"%s\"", var_name, value);

	HAL_UART_Transmit(&huart1, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

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


/**
 * Map a value from one range to another.
 *
 * @param input: The value to map.
 * @param in_min: The minimum of the input range.
 * @param in_max: The maximum of the input range.
 * @param out_min: The minimum of the output range.
 * @param out_max: The maximum of the output range.
 * @return The mapped value in the output range.
 */
int32_t map_rpmbar(int32_t input, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    if (in_max == in_min) {
        return out_min;
    }

    if (input < in_min) input = in_min;
    if (input > in_max) input = in_max;

    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const char* map_gear(int value)  {
    if (value < 0 || value > 13) {
        return "X";
    }

    switch (value) {
        case 0: return "R";
        case 1: return "N";
        case 2: return "1";
        case 3: return "2";
        case 4: return "3";
        case 5: return "4";
        case 6: return "5";
        case 7: return "6";
        case 8: return "7";
        case 9: return "8";
        case 10: return "9";
        case 11: return "10";
        case 12: return "11";
        case 13: return "12";

        default:
        	return "X";
    }
}

char* int_to_string(int value) {
    // Determine required buffer size (including null terminator)
    size_t buffer_size = snprintf(NULL, 0, "%d", value) + 1;

    // Allocate memory
    char *string = (char*)malloc(buffer_size);
    if (!string) {
        return NULL;  // Return NULL if allocation fails
    }

    // Format the integer into the allocated string
    snprintf(string, buffer_size, "%d", value);

    return string;  // Caller must free() this memory
}

void updateUserInput() {
	user_input_data.buttons = 0; // Clear all bits initially
	user_input_data.encoder_1 = 0;
	user_input_data.encoder_2 = 0;
	user_input_data.encoder_3 = 0;
	user_input_data.hall_analog_1 = 0;
	user_input_data.hall_analog_2 = 0;
	// Buttons
	if (!HAL_GPIO_ReadPin(GPIOA, BUTTON_1_PIN)) user_input_data.buttons |= (1 << 0);
	if (!HAL_GPIO_ReadPin(GPIOA, BUTTON_2_PIN)) user_input_data.buttons |= (1 << 1);
	if (!HAL_GPIO_ReadPin(GPIOA, BUTTON_3_PIN)) user_input_data.buttons |= (1 << 2);
	if (!HAL_GPIO_ReadPin(GPIOA, BUTTON_4_PIN)) user_input_data.buttons |= (1 << 3);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_5_PIN)) user_input_data.buttons |= (1 << 4);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_6_PIN)) user_input_data.buttons |= (1 << 5);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_7_PIN)) user_input_data.buttons |= (1 << 6);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_8_PIN)) user_input_data.buttons |= (1 << 7);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_9_PIN)) user_input_data.buttons |= (1 << 8);
	if (!HAL_GPIO_ReadPin(GPIOB, BUTTON_10_PIN)) user_input_data.buttons |= (1 << 9);

	if(adc_data_ready) {
		processADC();
	}

//	// Encoders
//	if (HAL_GPIO_ReadPin(GPIOB, L_ENC_PIN_A)) user_input_data.buttons |= (1 << 10);
//	if (HAL_GPIO_ReadPin(GPIOB, L_ENC_PIN_B)) user_input_data.buttons |= (1 << 11);
//	if (HAL_GPIO_ReadPin(GPIOB, C_ENC_PIN_A)) user_input_data.buttons |= (1 << 11);
//	if (HAL_GPIO_ReadPin(GPIOB, C_ENC_PIN_B)) user_input_data.buttons |= (1 << 10);
//	if (HAL_GPIO_ReadPin(GPIOB, R_ENC_PIN_A)) user_input_data.buttons |= (1 << 11);
//	if (HAL_GPIO_ReadPin(GPIOB, R_ENC_PIN_B)) user_input_data.buttons |= (1 << 11);
}

void Start_ADC_DMA() {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_CHANNEL_COUNT);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adc_data_ready = 1;  // Set flag (do NOT process here)
    }
}

void processADC() {
	adc_data_ready = 0;

	// Convert ADC values to 8-bit format
	user_input_data.hall_analog_1 = map_hall_sensor(adc_values[2]);
	user_input_data.hall_analog_2 = map_hall_sensor(adc_values[3]);

	// Process hall button thresholds
	if (adc_values[0] > 2200) user_input_data.buttons |= (1 << 10);
	if (adc_values[1] > 2200) user_input_data.buttons |= (1 << 11);
}
uint8_t map_hall_sensor(uint16_t adc_value) {
    if (adc_value < ADC_REST) adc_value = ADC_REST;
    if (adc_value > ADC_MAX) adc_value = ADC_MAX;

    return (uint8_t)(((adc_value - ADC_REST) * 255) / (ADC_MAX - ADC_REST));
}

/*
 * CAN BUS FUNCTIONS
 */
void CAN_Transmit() {
	uint32_t currentTime = HAL_GetTick();

	if(currentTime - lastSendTime >= 20) {
		CAN_TxHeaderTypeDef TxHeader;
		uint32_t TxMailbox;

		// Create a telemetry_packet instance and initialize its fields
		user_input_data_t dataToSend = user_input_data;
//		dataToSend.buttons = 0x0F0F;         // Example: Buttons pressed
//		dataToSend.hall_analog_1 = 100;      // Example: Hall sensor 1 value
//		dataToSend.hall_analog_2 = 200;      // Example: Hall sensor 2 value
//		dataToSend.encoder_1 = 1000;         // Example: Encoder 1 value
//		dataToSend.encoder_2 = -2000;        // Example: Encoder 2 value
//		dataToSend.encoder_3 = 5000;         // Example: Encoder 3 value

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
		        HAL_CAN_Stop(&hcan);  // Stop CAN
		        HAL_CAN_Start(&hcan); // Restart CAN

		        // Optional: Clear error flags
		        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_ERRI);
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
    static uint32_t last_receive_time_1 = 0;
    // Receive the message
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK) {
    	 // Check if the message ID matches 0x100
		if (rxHeader.StdId == 0x100) {
			static uint8_t buffer[sizeof(telemetry_packet)];
			static uint8_t offset = 0;
			last_receive_time_1 = HAL_GetTick();
			// Copy received data into buffer
			uint8_t bytesToCopy = (rxHeader.DLC < sizeof(telemetry_packet) - offset) ? rxHeader.DLC : sizeof(telemetry_packet) - offset;
			memcpy(&buffer[offset], rxData, bytesToCopy);
			offset += bytesToCopy;

			// Check if the entire packet has been received
			if (offset >= sizeof(telemetry_packet)) {
				// Copy buffer into the telemetry_packet struct
				memcpy(&telemetry_data, buffer, sizeof(telemetry_packet));
				offset = 0; // Reset offset for the next packet
			}
			if (HAL_GetTick() - last_receive_time_1 > 500) {
				printf("CAN data timeout: Resetting buffer!\n");
				offset = 0;  // Prevent infinite accumulation
			}
		}
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
