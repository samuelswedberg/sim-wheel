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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cJSON.h>
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include <time.h>
#include <usart.h>
#include <tim.h>
#include <spi.h>
#include <adc.h>
#include <usb_otg.h>
#include <usbd_hid_custom_if.h>
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

typedef struct __attribute__((packed)){
	uint8_t steering;          // Steering Wheel (0-255) x axis
	uint8_t throttle;          // Throttle (0-255) z axis
	uint8_t brake;             // Brake (0-255) Rx axis
	uint8_t clutch;            // Clutch (0-255) Ry axis
	uint32_t buttons;          // Buttons (16 bits) 32 bits
	uint8_t rz;         // Encoder 1 (Rz axis)
	uint8_t slider;     // Encoder 2 (Slider axis)
} __attribute__((packed)) HIDReport_t;

HIDReport_t HIDReport;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_RESOLUTION 2400  // Example: number of counts per rotation
#define WHEEL_MAX_ANGLE 450.0f      // Maximum angle for the lock (degrees)
#define BUFFER_SIZE 256
#define MAX_REVOLUTIONS 2
#define ADC_RESOLUTION 4096
#define ADC_MAX_VOLTAGE 5.0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t tx_buffer[BUFFER_SIZE];  // Buffer to hold received data
uint8_t gCommandData[BUFFER_SIZE];  // Buffer to hold a copy of the received command

float gPWM;
float gPWMConst;
float gTotalforce;
volatile int16_t gPosition;
uint8_t gDir;
float wheel_angle = 0.0;
float angular_velocity = 0.0;
float gDelta;

float gBrake = 0;
float gAccel = 0;
float gSteering = 0;

float hall_voltage = 0;
float max_hall_voltage = 0;
uint32_t max_position = 0;

/*
 * Default strength is 0.5 (results in bell curve feedback)
 * Over drive would be greater than 0.5
 * example: Strength at 1 makes PWM reach 255 at ffbSignal at 0.5 from the game.
 */
float gStrength = 0.1;
float gFfbSignal;
static float last_encoder_count = 0;
static float last_update_time = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ControlLoopTaskHandle;
osSemaphoreId spiSendMutexHandle;
osSemaphoreId uartMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void runUART();
void runSPI();
void runReport();
float constrain(float x, float lower, float upper);
float calculate_inertia(float force_feedback, float angular_velocity);
float calculate_damping(float angular_velocity);
float calculate_friction(float angular_velocity);
float calculate_lock(float angle);
float scale_to_pwm(float total_force);
extern void init_encoder();
int16_t read_encoder_position();
void reset_encoder_position();
float get_angle_degrees();
void update_wheel_position_and_velocity(float *wheel_angle, float *angular_velocity);
void set_motor_pwm(float pwm_value);
void set_motor_direction(uint8_t direction);
uint8_t map_wheel_position_to_axis(int32_t position);
void motor_rotate_left();
void motor_rotate_right();
float read_hall_sensor();
void move_to_position(uint32_t target_position);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlLoop(void const * argument);

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
	telemetry_data.tRpm = 0;
	telemetry_data.tRpm = 0;
	telemetry_data.tSpeedKmh = 0;
	telemetry_data.tHasDRS = 0;
	telemetry_data.tDrs = 0;
	telemetry_data.tPitLim = 0;
	telemetry_data.tFuel = 0;
	telemetry_data.tBrakeBias = 0;
	gFfbSignal = 0;

	HIDReport.steering = 0;        // Steering data (0-255)
	HIDReport.throttle = 0;        // Throttle data (0-255)
	HIDReport.brake = 0;           // Brake data (0-255)
	HIDReport.clutch = 0;         // Clutch data (0-255)
	HIDReport.buttons = 0;   // Each bit represents a button'
	HIDReport.rz = 0;
	HIDReport.slider = 0;
	memset(&telemetry_data, 0, sizeof(telemetry_packet)); // Zero-initialize
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of spiSendMutex */
  osSemaphoreDef(spiSendMutex);
  spiSendMutexHandle = osSemaphoreCreate(osSemaphore(spiSendMutex), 1);

  /* definition and creation of uartMutex */
  osSemaphoreDef(uartMutex);
  uartMutexHandle = osSemaphoreCreate(osSemaphore(uartMutex), 1);

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

  /* definition and creation of ControlLoopTask */
  osThreadDef(ControlLoopTask, StartControlLoop, osPriorityHigh, 0, 128);
  ControlLoopTaskHandle = osThreadCreate(osThread(ControlLoopTask), NULL);

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
	  //gFfbSignal = oscillate();
	  // Define the proportional gain (adjust this value to change stiffness)
//	  const float Kp = 1.0f;
//	  const float deadband = 50.0f; // Deadband range in degrees
//	  /* Infinite loop */
//	  for(;;)
//	  {
//		  float error = wheel_angle;
//
//		  if (fabsf(error) < deadband) {
//		      gFfbSignal = 0.0f;
//		  } else {
//		      gFfbSignal = -Kp * (error / WHEEL_MAX_ANGLE);
//		      gFfbSignal = constrain(gFfbSignal, -1.0f, 1.0f);
//		  }
//		  // Small delay to allow other tasks to run
//		  osDelay(10);
//	  }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlLoop */
/**
* @brief Function implementing the ControlLoopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlLoop */
void StartControlLoop(void const * argument)
{
  /* USER CODE BEGIN StartControlLoop */
  /* Infinite loop */
  for(;;)
  {
	  float total_force = 0.0;
	  const float Kp = 1.0f;

	  for (;;) {
		  // Step 1: Retrieve current force feedback signal (e.g., from game data).
		  float force_feedback_signal = gFfbSignal;

		  // Step 2: Calculate individual forces based on physics:
		  float inertia_force = calculate_inertia(force_feedback_signal, angular_velocity);
		  float damping_force = calculate_damping(angular_velocity);
		  float friction_force = calculate_friction(angular_velocity);
		  float lock_force = calculate_lock(wheel_angle);

		  // Step 3: Sum all forces and scale to PWM range:
		  total_force = force_feedback_signal + inertia_force + damping_force + friction_force + lock_force;

		  // Strength gain
		  total_force *= gStrength;

		  total_force = constrain(total_force, -1.0, 1.0);

		  // Deadband
		  const float FORCE_DEADBAND_THRESHOLD = 0.05f; // Adjust as needed
		  const float ANGLE_DEADBAND_THRESHOLD = 50.0f; // Adjust as needed
		  float error = wheel_angle;
		  if (fabsf(error) < ANGLE_DEADBAND_THRESHOLD)  {
			  total_force = 0.0f;
		  } else { // TEST CODE: gives increasing feedback farther away from center
			  total_force = -Kp * (error / WHEEL_MAX_ANGLE);
			  total_force = constrain(total_force, -1.0f, 1.0f);
		  }

		  // Step 4: Map total_force to PWM and determine direction
		  float pwm_output = scale_to_pwm(total_force);
		  // 0 is negative direction; 1 is positive direction
		  uint8_t motor_direction = (total_force >= 0) ? 1 : 0;

		  // Debug
		  gDir = motor_direction;
		  gTotalforce = total_force;
		  gPWMConst = pwm_output;

		  // Step 5: Send PWM signal to H-bridge for motor control:
		  set_motor_direction(motor_direction);
		  set_motor_pwm(pwm_output);

		  // Step 6: Update wheel position and velocity for next loop:
		  update_wheel_position_and_velocity(&wheel_angle, &angular_velocity);

		  if (osSemaphoreWait(uartMutexHandle, osWaitForever) == osOK) {
			  runUART();
		  }

		  if (osSemaphoreWait(spiSendMutexHandle, osWaitForever) == osOK) {
			  runSPI();
		  }

		  runReport();

		  // Run this task periodically (every 10ms):
		  osDelay(10);
	  }
  }
  /* USER CODE END StartControlLoop */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
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

void runReport() {
	HIDReport.steering = gSteering;        // Steering data (0-255)
	HIDReport.throttle = gAccel;        // Throttle data (0-255)
	HIDReport.brake = gBrake;           // Brake data (0-255)
	HIDReport.clutch = 0;         // Clutch data (0-255)
	HIDReport.buttons = 0;   // Each bit represents a button'
	HIDReport.rz = 0;
	HIDReport.slider = 0;
	USBD_CUSTOM_HID_SendCustomReport((uint8_t *)&HIDReport, sizeof(HIDReport));
}

void runSPI() {
	HAL_StatusTypeDef status;
	uint8_t buffer[sizeof(telemetry_packet)];
	telemetry_packet dataToSend = {3600, 1, 120, 0, 0, 0, 45, 0}; // DEBUG DATA
	memcpy(&buffer, (uint8_t*)&telemetry_data, sizeof(telemetry_packet));

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set NSS low

	status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)&dataToSend, sizeof(telemetry_packet)); // DEBUG DATA

	//status = HAL_SPI_Transmit_DMA(&hspi2, (uint8_t*)&buffer, sizeof(telemetry_packet)); // REAL DATA

	//uint8_t data = 0x0F;  // Test byte
	//status = HAL_SPI_Transmit_DMA(&hspi2, &data, 1);

	//uint8_t testData[4] = {0xAA, 0xBB, 0xCC, 0xDD}; // tRpm = 3600 in little-endian
	//HAL_SPI_Transmit_DMA(&hspi2, &testData, sizeof(testData));

	// Check for errors
	if (status != HAL_OK) {
		send_response("SPI Transmission Error");
	}
	// Wait for transmission to complete (optional but safer)
}

void runUART() {
	// Wait for notification from UART callback
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	// Process the command received via UART
	process_command(gCommandData);

	// Introduce a delay if necessary
	vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay as needed
	// Re-enable UART reception
	HAL_UART_Receive_IT(&huart2, rx_buffer, sizeof(rx_buffer));
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
    const float MIN_PWM = 50.0f;    // Minimum PWM value for the motor to start moving
    const float MAX_PWM = 255.0f;   // Maximum PWM value

    // If total_force is zero, return zero PWM output
    if (total_force == 0.0f) {
        gPWM = 0.0f;
        return 0.0f;
    }

    // Calculate PWM output
    float pwm_output = fabs(total_force) * (MAX_PWM - MIN_PWM) + MIN_PWM;

    // Constrain PWM output to valid range
    pwm_output = constrain(pwm_output, MIN_PWM, MAX_PWM);

    // Update debug variable
    gPWM = pwm_output;

    return pwm_output;
}

uint8_t map_wheel_position_to_axis(int32_t position) {
    int32_t min_position = -450;
    int32_t max_position = 450;

    // Clamp the position to the valid range
	if (position < min_position) {
		position = min_position;
	} else if (position > max_position) {
		position = max_position;
	}

	// Reverse the mapping
	return (uint8_t)((((max_position - position) * 255) + (max_position - min_position) / 2) / (max_position - min_position));
}

extern void init_encoder() {
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
    float dt = (current_time - last_update_time) / 1000.0f;  // Convert ms to seconds

    // Calculate the change in angle
    float delta_angle = current_angle - last_encoder_count;

    // Implement a threshold to ignore small changes
    if (fabs(delta_angle) < 0.25f) {  // Adjust the threshold as needed
        delta_angle = 0.0f;
    }

    // Update the wheel angle, keeping within the lock limit
    *wheel_angle += delta_angle;
    if (*wheel_angle > WHEEL_MAX_ANGLE) *wheel_angle = WHEEL_MAX_ANGLE;
    if (*wheel_angle < -WHEEL_MAX_ANGLE) *wheel_angle = -WHEEL_MAX_ANGLE;

    gDelta = delta_angle;
    // Calculate angular velocity (degrees per second)
    if (dt > 0.0001f) {  // Avoid division by zero
        *angular_velocity = delta_angle / dt;
    } else {
        *angular_velocity = 0.0f;
    }

    // Store the current values for the next update
    last_encoder_count = current_angle;
    last_update_time = current_time;

    gSteering = map_wheel_position_to_axis(*wheel_angle);
}


void set_motor_pwm(float pwm_value) {
    // Assuming pwm_value ranges from 0 to 255
    uint32_t pulse = (uint32_t)((pwm_value / 255.0) * htim3.Init.Period);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
}

void set_motor_direction(uint8_t direction) {
    if (direction == 1) { // Forward
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // IN1 = HIGH
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // IN2 = LOW
    } else if (direction == 0) { // Reverse
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // IN1 = LOW
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // IN2 = HIGH
    } else { // Stop
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1 = LOW
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2 = LOW
    }
}

extern void releaseSPI() {
	// Process the received data (rx_buffer)
	memcpy(gCommandData, rx_buffer, sizeof(rx_buffer));
	osSemaphoreRelease(spiSendMutexHandle);
	// Clear the buffer for the next message
	memset(rx_buffer, 0, BUFFER_SIZE);
}

extern void signalTelemetryTask() {
    // Pull CS line high to deselect the slave
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    DWT_Delay_us(2);
    osSemaphoreRelease(uartMutexHandle);
}

extern void restartUart(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(huart, rx_buffer, sizeof(rx_buffer));
}

void motor_rotate_left() {
    uint32_t target_position = gPosition - (MAX_REVOLUTIONS * ENCODER_RESOLUTION);
    while (gPosition > target_position) {
        // Set motor speed and direction
        set_motor_direction(1);
        set_motor_pwm(100);

        // Read Hall Sensor
        hall_voltage = read_hall_sensor();
        if (hall_voltage > max_hall_voltage) {
            max_hall_voltage = hall_voltage;
            max_position = gPosition;
        }
    }

    // Stop Motor
    set_motor_pwm(0);
}

void motor_rotate_right() {
    uint32_t target_position = gPosition + (MAX_REVOLUTIONS * ENCODER_RESOLUTION);
    while (gPosition < target_position) {
        // Set motor speed and direction
    	set_motor_direction(0);
		set_motor_pwm(100);

        // Read Hall Sensor
        hall_voltage = read_hall_sensor();
        if (hall_voltage > max_hall_voltage) {
            max_hall_voltage = hall_voltage;
            max_position = gPosition;
        }
    }

    // Stop Motor
    set_motor_pwm(0);
}

float read_hall_sensor() {
    uint32_t adc_value = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return (adc_value * ADC_MAX_VOLTAGE) / ADC_RESOLUTION;
}

void move_to_position(uint32_t target_position) {
    while (gPosition != target_position) {
        if (gPosition < target_position) {
            // Rotate Right
        	set_motor_direction(1);
			set_motor_pwm(100);
        } else {
            // Rotate Left
        	set_motor_direction(0);
			set_motor_pwm(100);
        }
    }

    // Stop Motor
    set_motor_pwm(0);
}
/* USER CODE END Application */
