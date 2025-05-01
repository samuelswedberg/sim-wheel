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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <neopixel.h>
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
    uint32_t buttons;       // 10 physical buttons + 2 hall sensor buttons (12 total, packed into 16 bits)
    uint8_t hall_analog_1;  // First hall sensor analog value (0-255)
    uint8_t hall_analog_2;  // Second hall sensor analog value (0-255)
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

#define L_ENC_PIN_CLK GPIO_PIN_7 // PB7
#define L_ENC_PIN_DT GPIO_PIN_6 // PB6
#define C_ENC_PIN_CLK GPIO_PIN_5 // PB5
#define C_ENC_PIN_DT GPIO_PIN_4 // PB4
#define R_ENC_PIN_CLK GPIO_PIN_3 // PB3
#define R_ENC_PIN_DT GPIO_PIN_15 // PA15

#define NEOPIXEL_PIN GPIO_PIN_8
#define NUM_LEDS 8 // MATCH IN neopixel.c

#define ADC_CHANNEL_COUNT 4
#define ADC_REST 2000
#define ADC_MAX 2800

#define ENCODER_HOLD_DURATION 500  // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId nextionTaskHandle;
osThreadId canTaskHandle;
/* USER CODE BEGIN PV */
// Oscillation state variables
int value = 0;
int direction = 1; // 1 for increasing, -1 for decreasing
int step = 1;      // Increment/Decrement step
int delay_ms = 25; // Delay between updates

int enc_l_flag = 0;
int enc_c_flag = 0;
int enc_r_flag = 0;

uint32_t enc_l_time = 0, enc_c_time = 0, enc_r_time = 0;

int shift_paddle_l = 0;
int shift_paddle_r = 0;

uint32_t lastSendTime = 0;
uint16_t adc_values[ADC_CHANNEL_COUNT];

volatile uint8_t adc_data_ready = 0;
volatile uint32_t last_enc_interrupt_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);
void startNextionTask(void const * argument);
void startCanTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int oscillate_value();
void Flash_Onboard_LED(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t delay_ms);
void CAN_Transmit();
void sendCANMessage(uint16_t canID, int32_t value);
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
void updateNeopixels();
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
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

  /* definition and creation of nextionTask */
  osThreadDef(nextionTask, startNextionTask, osPriorityAboveNormal, 0, 192);
  nextionTaskHandle = osThreadCreate(osThread(nextionTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, startCanTask, osPriorityHigh, 0, 192);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vTaskStartScheduler();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
	telemetry_data.tRpm = 0;
	telemetry_data.tRpm = 0;
	telemetry_data.tSpeedKmh = 0;
	telemetry_data.tHasDRS = 0;
	telemetry_data.tDrs = 0;
	telemetry_data.tPitLim = 0;
	telemetry_data.tFuel = 0;
	telemetry_data.tBrakeBias = 0;


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
  filterConfig.FilterMaskIdHigh = 0x700 << 5;;   // Accept all IDs
  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;  // Assign to FIFO 1
  filterConfig.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filterConfig);
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB4 PB6 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
	int mappedRpmBar = map_rpmbar(telemetry_data.tRpm, 0, telemetry_data.tMaxRpm, 0, 100);
	char *mappedRpm = int_to_string(telemetry_data.tRpm);
	char *mappedGear = map_gear(telemetry_data.tGear);
	char *mappedSpeed = int_to_string(telemetry_data.tSpeedKmh);
	char *mappedHasDrs = int_to_string(telemetry_data.tHasDRS);
	char *mappedPitLim = int_to_string(telemetry_data.tPitLim);
	char *mappedFuel = int_to_string(telemetry_data.tFuel);
	char *mappedBrakeBias = int_to_string(telemetry_data.tBrakeBias);

	send_int_to_nextion("rpmbar", mappedRpmBar);
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

void updateNeopixels() {

    // Prevent divide by zero
    if (telemetry_data.tMaxRpm == 0) return;

    // Clamp currentRPM to maxRPM
    if (telemetry_data.tRpm > telemetry_data.tMaxRpm) telemetry_data.tRpm = telemetry_data.tMaxRpm;

    // Calculate how many LEDs to light
    uint8_t ledsToLight = (telemetry_data.tRpm * NUM_LEDS) / telemetry_data.tMaxRpm;

    // Loop through and set color
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        if (i < ledsToLight) {
            // Green at low RPM, shift to red near redline (optional)
            uint8_t r = (i * 255) / NUM_LEDS;    // Red increases with RPM
            uint8_t g = 255 - r;                // Green fades out
            neopixel_set(i, r, g, 0);
        } else {
            neopixel_set(i, 0, 0, 0); // Turn off
        }
    }

    neopixel_show();
}

void updateUserInput() {
	uint32_t now = HAL_GetTick();
	user_input_data.buttons = 0; // Clear all bits initially
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

	if (enc_l_flag == 1) user_input_data.buttons |= (1 << 12);
	if (enc_l_flag == -1) user_input_data.buttons |= (1 << 13);
	if (enc_c_flag == 1) user_input_data.buttons |= (1 << 14);
	if (enc_c_flag == -1) user_input_data.buttons |= (1 << 15);
	if (enc_r_flag == 1) user_input_data.buttons |= (1 << 16);
	if (enc_r_flag == -1) user_input_data.buttons |= (1 << 17);

	if (now - enc_l_time > ENCODER_HOLD_DURATION) enc_l_flag = 0;
	if (now - enc_c_time > ENCODER_HOLD_DURATION) enc_c_flag = 0;
	if (now - enc_r_time > ENCODER_HOLD_DURATION) enc_r_flag = 0;
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

	int paddleThreshold = 100;

	// Convert ADC values to 8-bit format
	user_input_data.hall_analog_1 = map_hall_sensor(adc_values[2]);
	user_input_data.hall_analog_2 = map_hall_sensor(adc_values[3]);

	if (shift_paddle_l == 0 && shift_paddle_r == 0)
	{
		shift_paddle_l = adc_values[0];
		shift_paddle_r = adc_values[1];
	}
	else
	{
		// Process hall button thresholds
		if (shift_paddle_l - adc_values[1] > paddleThreshold) user_input_data.buttons |= (1 << 11);
		if (shift_paddle_r - adc_values[0] > paddleThreshold) user_input_data.buttons |= (1 << 10);
	//	if (adc_values[0] > 2200) user_input_data.buttons |= (1 << 10);
	//	if (adc_values[1] > 2200) user_input_data.buttons |= (1 << 11);
	}

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

	if(currentTime - lastSendTime >= 2) {
		sendCANMessage(0x200, (int32_t)user_input_data.buttons);
		sendCANMessage(0x201, (int32_t)user_input_data.hall_analog_1 & 0xFF);
		sendCANMessage(0x202, (int32_t)user_input_data.hall_analog_2 & 0xFF);

		lastSendTime = currentTime;  // Update last transmission time
		HAL_Delay(1);
	}
}


void sendCANMessage(uint16_t canID, int32_t value) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];  // 4-byte buffer

    // Ensure correct byte order in CAN message
    TxData[0] = (uint8_t)(value & 0xFF);
    TxData[1] = (uint8_t)((value >> 8) & 0xFF);
    TxData[2] = (uint8_t)((value >> 16) & 0xFF);
    TxData[3] = (uint8_t)((value >> 24) & 0xFF);

    uint32_t TxMailbox;

    // Configure the CAN header
    TxHeader.StdId = canID;  // Set the ID
    TxHeader.IDE = CAN_ID_STD;  // Standard 11-bit ID
    TxHeader.RTR = CAN_RTR_DATA;  // Data frame, not remote request
    TxHeader.DLC = sizeof(value);  // Data Length = 4 bytes

    // Copy integer value into TxData buffer (ensure correct byte order)
    memcpy(TxData, &value, sizeof(value));

    // Send the CAN message
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    	 // Optionally log the state of CAN error counters
		uint32_t error = HAL_CAN_GetError(&hcan);
		HAL_CAN_Stop(&hcan);  // Stop CAN
		HAL_CAN_Start(&hcan); // Restart CAN
		// Optional: Clear error flags
		__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_ERRI);
    }
}

// CAN receive interrupt callback
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);

		int32_t value;
		memcpy(&value, RxData, sizeof(value));

		switch (RxHeader.StdId) {
			// Wheelbase
			case 0x100: telemetry_data.tRpm = value; break;
			case 0x101: telemetry_data.tGear = value; break;
			case 0x102: telemetry_data.tSpeedKmh = value; break;
			case 0x103: telemetry_data.tHasDRS = value; break;
			case 0x104: telemetry_data.tDrs = value; break;
			case 0x105: telemetry_data.tPitLim = value; break;
			case 0x106: telemetry_data.tFuel = value; break;
			case 0x107: telemetry_data.tBrakeBias = value; break;
			case 0x108: telemetry_data.tMaxRpm = value; break;
			case 0x109: telemetry_data.tForceFB = value; break;
			default: break;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t current_time = HAL_GetTick();  // Get system time in ms
	if (current_time - last_enc_interrupt_time < 250) return;  // Debounce (5ms)
	last_enc_interrupt_time = current_time;
    if (GPIO_Pin == L_ENC_PIN_CLK) {
        if (HAL_GPIO_ReadPin(GPIOB, L_ENC_PIN_DT) == GPIO_PIN_SET) {
        	enc_l_flag = 1;
        } else {
        	enc_l_flag = -1;
        }
        enc_l_time = current_time;
    }
    if (GPIO_Pin == C_ENC_PIN_CLK) {
		if (HAL_GPIO_ReadPin(GPIOB, C_ENC_PIN_DT) == GPIO_PIN_SET) {
			enc_c_flag = 1;
		} else {
			enc_c_flag = -1;
		}
		enc_c_time = current_time;
	}
    if (GPIO_Pin == R_ENC_PIN_CLK) {
		if (HAL_GPIO_ReadPin(GPIOA, R_ENC_PIN_DT) == GPIO_PIN_SET) {
			enc_r_flag = 1;
		} else {
			enc_r_flag = -1;
		}
		enc_r_time = current_time;
	}
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
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // Turn LED off
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // Turn LED off
	  osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startNextionTask */
/**
* @brief Function implementing the nextionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startNextionTask */
void startNextionTask(void const * argument)
{
  /* USER CODE BEGIN startNextionTask */
  /* Infinite loop */
//	neopixel_clear();
//	neopixel_show();
//	osDelay(10);

  for(;;)
  {
	  updateTelemetry();
/*	  updateNeopixels();*/
//	  neopixel_set(0, 255, 0, 0); // RED
//	  neopixel_set(1, 0, 255, 0); // GREEN
//	  neopixel_set(2, 0, 0, 255); // BLUE
//	  neopixel_set(3, 255, 255, 255); // WHITE
//	  neopixel_show();

	  osDelay(1);
  }
  /* USER CODE END startNextionTask */
}

/* USER CODE BEGIN Header_startCanTask */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startCanTask */
void startCanTask(void const * argument)
{
  /* USER CODE BEGIN startCanTask */
  /* Infinite loop */
	Start_ADC_DMA();
  for(;;)
  {
	  updateUserInput();
	  CAN_Transmit();
	  osDelay(5);
  }
  /* USER CODE END startCanTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
