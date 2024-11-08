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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cJSON.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// !!! MUST MATCH PICO STRUCT
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

telemetry_packet telemetry_data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 256
#define ENCODER_RESOLUTION 2400  // Example: number of counts per rotation
#define WHEEL_MAX_ANGLE 450      // Maximum angle for the lock (degrees)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
TaskHandle_t spiTaskHandle;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t tx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t gCommandData[BUFFER_SIZE];  // Buffer to hold a copy of the received command

float gFfbSignal;
float gPWM;
float gPWMConst;
float gTotalforce;
int16_t gPosition;

/*
 * Default strength is 0.5 (results in bell curve feedback)
 * Over drive would be greater than 0.5
 * example: Strength at 1 makes PWM reach 255 at ffbSignal at 0.5 from the game.
 */
float gStrength = 0.5;

static int32_t last_encoder_count = 0;
static uint32_t last_update_time = 0;

extern osSemaphoreId spiSendSemaphoreHandle;
extern osThreadId telemetryTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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

		if (cJSON_IsNumber(forceFB)) { gFfbSignal = (float)forceFB->valuedouble; }
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

// Initialize DWT for cycle counting
void DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0; // Reset the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter
}

// Delay function using DWT for accurate timing in microseconds
void DWT_Delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000); // Convert microseconds to ticks

    while ((DWT->CYCCNT - startTick) < delayTicks) {
        // Wait until the required delay has passed
    }
}

float oscillate() {
    float period = 3.0;  // Oscillation period in seconds
    float elapsed_time = HAL_GetTick() / 1000.0;  // Convert milliseconds to seconds

    // Calculate the oscillation value using a sine wave
    return sin((2 * M_PI * elapsed_time) / period);
}

float constrain(float x, float lower, float upper) {
    if (x < lower) return lower;
    if (x > upper) return upper;
    return x;
}

float calculate_inertia(float force_feedback, float angular_velocity) {
    static float previous_output = 0;
    float inertia_coefficient = 0.1; // Fine-tune for feel
    float inertia_force = inertia_coefficient * previous_output + (1 - inertia_coefficient) * force_feedback;
    previous_output = inertia_force;
    return inertia_force;
}

float calculate_damping(float angular_velocity) {
    float damping_coefficient = 0.05;
    return -damping_coefficient * angular_velocity;
}

float calculate_friction(float angular_velocity) {
    float friction_coefficient = 0.02;
    if (angular_velocity > 0) {
        return -friction_coefficient;
    } else if (angular_velocity < 0) {
        return friction_coefficient;
    } else {
        return 0;
    }
}

float calculate_lock(float angle) {
    float lock_coefficient = 1.0;
    float max_angle = 450.0;
    if (angle > max_angle) {
        return -lock_coefficient * (angle - max_angle);
    } else if (angle < -max_angle) {
        return -lock_coefficient * (angle + max_angle);
    }
    return 0;
}

float scale_to_pwm(float total_force) {
	// Map total_force from -1..1 to 0..255
	float pwm_output = fabs(total_force) * 255.0;
	gPWM = pwm_output;
	return constrain(pwm_output, 0, 255.0);
}

void init_encoder() {
    // Start the encoder mode timer
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    reset_encoder_position();
}

int16_t read_encoder_position() {
    return __HAL_TIM_GET_COUNTER(&htim2);  // Get the current encoder count
}

void reset_encoder_position() {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset the encoder count to zero
}

float get_angle_degrees() {
    int16_t position = read_encoder_position();
    gPosition = position;
    return (position * 360.0) / ENCODER_RESOLUTION;
}

void update_wheel_position_and_velocity(float *wheel_angle, float *angular_velocity) {
    // Get the current encoder count
    float current_angle = get_angle_degrees();

    // Calculate time difference (in seconds) since the last update
    uint32_t current_time = HAL_GetTick();  // In milliseconds
    float dt = (current_time - last_update_time) / 1000.0;  // Convert ms to seconds

    // Calculate the change in angle
    float delta_angle = current_angle - last_encoder_count;

    // Update the wheel angle, keeping within the lock limit
    *wheel_angle += delta_angle;
    if (*wheel_angle > WHEEL_MAX_ANGLE) *wheel_angle = WHEEL_MAX_ANGLE;
    if (*wheel_angle < -WHEEL_MAX_ANGLE) *wheel_angle = -WHEEL_MAX_ANGLE;

    // Calculate angular velocity (degrees per second)
    if (dt > 0) {
        *angular_velocity = delta_angle / dt;
    } else {
        *angular_velocity = 0;
    }

    // Store the current values for the next update
    last_encoder_count = current_angle;
    last_update_time = current_time;
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
  MX_USB_DEVICE_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */

  telemetry_data.tRpm = 0;
  telemetry_data.tRpm = 0;
  telemetry_data.tSpeedKmh = 0;
  telemetry_data.tHasDRS = 0;
  telemetry_data.tDrs = 0;
  telemetry_data.tPitLim = 0;
  telemetry_data.tFuel = 0;
  telemetry_data.tBrakeBias = 0;
  gFfbSignal = 0;
  memset(&telemetry_data, 0, sizeof(telemetry_packet)); // Zero-initialize
  init_encoder();
  DWT_Init();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Pull CS line high to deselect the slave
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    DWT_Delay_us(2);
    osSemaphoreRelease(spiSendSemaphoreHandle);
}

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

/* USER CODE END 4 */

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
