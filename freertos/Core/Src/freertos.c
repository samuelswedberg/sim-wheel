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
#include <tim.h>
#include <can.h>
#include <adc.h>
#include <usb_otg.h>
#include <usbd_hid_custom_if.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// !!! STRUCTS MUST MATCH ACROSS DEVICES !!!
typedef struct __attribute__((packed)){
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

typedef struct __attribute__((packed, aligned(1))) {
    uint32_t buttons;       // 10 physical buttons + 2 hall sensor buttons (12 total, packed into 16 bits)
    uint8_t hall_analog_1;  // First hall sensor analog value (0-255)
    uint8_t hall_analog_2;  // Second hall sensor analog value (0-255)
} user_input_data_t;

user_input_data_t user_input_data;

typedef struct __attribute__((packed, aligned(1))) {
    int16_t encoder_1;      // First encoder value
    int16_t encoder_2;      // Second encoder value
    int16_t encoder_3;      // Third encoder value
} pedal_data_t;

pedal_data_t pedal_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_RESOLUTION 2400  // Example: number of counts per rotation
#define WHEEL_MAX_ANGLE 450.0f      // Maximum angle for the lock (degrees)
#define BUFFER_SIZE 256
#define MAX_REVOLUTIONS 2

#define ADC_CHANNEL_COUNT  1  // Only one channel used

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
float gPWMConstDebug = 0;
float gTotalforce;
volatile int16_t gPosition;
uint8_t gDir;
uint8_t gDirDebug = 1;
float wheel_angle = 0.0;
float angular_velocity = 0.0;
float gDelta;

float gBrake = 0;
float gAccel = 0;
float gSteering = 0;

float hall_voltage = 0;
float max_hall_voltage = 0;
float gHall = 0;
uint32_t max_position = 0;

int gDebugCounter1 = 0;
int gDebugCounter2 = 0;

uint32_t can_error = 0;
uint32_t lastSendTime = 0;

uint16_t adc_buffer[1];  // DMA needs a buffer
volatile uint8_t adc_data_ready = 0;

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
osThreadId CommLoopTaskHandle;
osSemaphoreId spiSendMutexHandle;
osSemaphoreId uartMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void runCAN();
void runReport();
void sendCANMessage(uint16_t canID, int32_t value);
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
void set_motor_pwm(float pwm_value, uint8_t direction);
//void set_motor_direction(uint8_t direction);
uint8_t map_wheel_position_to_axis(int32_t position);
void motor_rotate_left();
void motor_rotate_right();
void read_hall_sensor();
void Start_ADC_DMA();
void move_to_position(uint32_t target_position);
void processCAN();
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlLoop(void const * argument);
void StartCommLoopTask(void const * argument);

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
  osThreadDef(ControlLoopTask, StartControlLoop, osPriorityHigh, 0, 512);
  ControlLoopTaskHandle = osThreadCreate(osThread(ControlLoopTask), NULL);

  /* definition and creation of CommLoopTask */
  osThreadDef(CommLoopTask, StartCommLoopTask, osPriorityNormal, 0, 512);
  CommLoopTaskHandle = osThreadCreate(osThread(CommLoopTask), NULL);

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
	Start_ADC_DMA();
  for(;;)
  {
	  float total_force = 0.0;
	  const float Kp = 1.0f;

	  for (;;) {
		  // Step 1: Retrieve current force feedback signal (e.g., from game data).
		  float force_feedback_signal = telemetry_data.tForceFB;
		  read_hall_sensor();
		  // Step 2: Calculate individual forces based on physics:
//		  float inertia_force = calculate_inertia(force_feedback_signal, angular_velocity);
//		  float damping_force = calculate_damping(angular_velocity);
//		  float friction_force = calculate_friction(angular_velocity);
//		  float lock_force = calculate_lock(wheel_angle);

		  // Step 3: Sum all forces and scale to PWM range:
//		  total_force = force_feedback_signal + inertia_force + damping_force + friction_force + lock_force;
		  total_force = force_feedback_signal;
		  // Strength gain
		  total_force *= gStrength;

		  total_force = constrain(total_force, -1.0, 1.0);

		  // Deadband
		  const float FORCE_DEADBAND_THRESHOLD = 0.05f; // Adjust as needed
		  const float ANGLE_DEADBAND_THRESHOLD = 50.0f; // Adjust as needed
//		  float error = wheel_angle;
//		  if (fabsf(error) < ANGLE_DEADBAND_THRESHOLD)  {
//			  total_force = 0.0f;
//		  }
//		  else { // TEST CODE: gives increasing feedback farther away from center
//			  total_force = -Kp * (error / WHEEL_MAX_ANGLE);
//			  total_force = constrain(total_force, -1.0f, 1.0f);
//		  }

		  // Step 4: Map total_force to PWM and determine direction
		  float pwm_output = scale_to_pwm(total_force);
		  // 0 is negative direction; 1 is positive direction
		  uint8_t motor_direction = (total_force >= 0) ? 1 : 0;

		  // Debug
		  gDir = motor_direction;
		  gTotalforce = total_force;
		  gPWMConst = pwm_output;

		  // Step 5: Send PWM signal to H-bridge for motor control:
		  //set_motor_direction(motor_direction);
		  set_motor_pwm(pwm_output, motor_direction);
//		  set_motor_pwm(gPWMConstDebug, gDirDebug); //DEBUG MOTOR

		  // Step 6: Update wheel position and velocity for next loop:
		  update_wheel_position_and_velocity(&wheel_angle, &angular_velocity);

		  // Run this task periodically (every 10ms):
		  osDelay(5);
	  }
  }
  /* USER CODE END StartControlLoop */
}

/* USER CODE BEGIN Header_StartCommLoopTask */
/**
* @brief Function implementing the CommLoopTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommLoopTask */
void StartCommLoopTask(void const * argument)
{
  /* USER CODE BEGIN StartCommLoopTask */
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreWait(spiSendMutexHandle, 10) == osOK) {
		  runCAN();
	  }

	  runReport();
    osDelay(10);
  }
  /* USER CODE END StartCommLoopTask */
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

void runReport() {
	memset(&HIDReport, 0, sizeof(HIDReport_t));

	uint16_t max_clutch = pedal_data.encoder_3;

	if (user_input_data.hall_analog_1 > max_clutch) {
		max_clutch = user_input_data.hall_analog_1;
	}
	if (user_input_data.hall_analog_1 > max_clutch) {
		max_clutch = user_input_data.hall_analog_1;
	}

	HIDReport.steering = gSteering;
	HIDReport.throttle = pedal_data.encoder_1;
	HIDReport.brake = pedal_data.encoder_2;
	HIDReport.clutch = max_clutch;

	HIDReport.buttons = user_input_data.buttons;

//	HIDReport.rz = (uint8_t) (user_input_data.encoder_1 & 0xFF);
//	HIDReport.slider = (uint8_t) (user_input_data.encoder_2 & 0xFF);

	USBD_CUSTOM_HID_SendCustomReport((uint8_t *)&HIDReport, sizeof(HIDReport));
}

void runCAN() {
    static uint8_t messageIndex = 0;
    uint32_t currentTime = HAL_GetTick();

    if (currentTime - lastSendTime >= 2) {  // Reduce cycle time to avoid backlog
        switch (messageIndex) {
            case 0:
                sendCANMessage(0x100, telemetry_data.tRpm);
                sendCANMessage(0x101, telemetry_data.tGear);
                sendCANMessage(0x102, telemetry_data.tSpeedKmh);
                break;
            case 1:
                sendCANMessage(0x103, telemetry_data.tHasDRS);
                sendCANMessage(0x104, telemetry_data.tDrs);
                sendCANMessage(0x105, telemetry_data.tPitLim);
                break;
            case 2:
                sendCANMessage(0x106, telemetry_data.tFuel);
                sendCANMessage(0x107, telemetry_data.tBrakeBias);
                sendCANMessage(0x108, telemetry_data.tMaxRpm);
                break;
            case 3:
                sendCANMessage(0x109, telemetry_data.tForceFB);
                break;
        }

        messageIndex = (messageIndex + 1) % 4;  // Cycle through cases
        lastSendTime = currentTime;  // Update time
    }

    osSemaphoreRelease(spiSendMutexHandle);
}

void sendCANMessage(uint16_t canID, int32_t value) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];  // 4-byte buffer
    uint32_t TxMailbox;

    // Configure the CAN header
    TxHeader.StdId = canID;  // Set the ID
    TxHeader.IDE = CAN_ID_STD;  // Standard 11-bit ID
    TxHeader.RTR = CAN_RTR_DATA;  // Data frame, not remote request
    TxHeader.DLC = sizeof(value);  // Data Length = 4 bytes

    // Copy integer value into TxData buffer (ensure correct byte order)
    memcpy(TxData, &value, sizeof(value));

    // Send the CAN message
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    	 // Optionally log the state of CAN error counters
		uint32_t error = HAL_CAN_GetError(&hcan1);
		HAL_CAN_Stop(&hcan1);  // Stop CAN
		HAL_CAN_Start(&hcan1); // Restart CAN

		// Optional: Clear error flags
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_ERRI);
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
	if(gHall <= 1110 && gHall <= 1130)
	{
		reset_encoder_position();
	}
	int16_t position = read_encoder_position();
    gPosition = position;
    return (position * 360.0) / ENCODER_RESOLUTION;
}

//void update_wheel_position_and_velocity(float *wheel_angle, float *angular_velocity) {
//    // Get the current encoder count
//    float current_angle = get_angle_degrees();
//
//    // Calculate time difference (in seconds) since the last update
//    uint32_t current_time = HAL_GetTick();  // In milliseconds
//    float dt = (current_time - last_update_time) / 1000.0f;  // Convert ms to seconds
//
//    // Calculate the change in angle
//    float delta_angle = current_angle - last_encoder_count;
//
//    // Implement a threshold to ignore small changes
//    if (fabs(delta_angle) < 0.25f) {  // Adjust the threshold as needed
//        delta_angle = 0.0f;
//    }
//
//    // Update the wheel angle, keeping within the lock limit
//    *wheel_angle += delta_angle;
//    if (*wheel_angle > WHEEL_MAX_ANGLE) *wheel_angle = WHEEL_MAX_ANGLE;
//    if (*wheel_angle < -WHEEL_MAX_ANGLE) *wheel_angle = -WHEEL_MAX_ANGLE;
//
//    gDelta = delta_angle;
//    // Calculate angular velocity (degrees per second)
//    if (dt > 0.0001f) {  // Avoid division by zero
//        *angular_velocity = delta_angle / dt;
//    } else {
//        *angular_velocity = 0.0f;
//    }
//
//    // Store the current values for the next update
//    last_encoder_count = current_angle;
//    last_update_time = current_time;
//
//    gSteering = map_wheel_position_to_axis(*wheel_angle);
//}

void update_wheel_position_and_velocity(float *wheel_angle, float *angular_velocity) {
    // Get the absolute current encoder angle
    float current_angle = get_angle_degrees();

    // Clamp the wheel angle between -WHEEL_MAX_ANGLE and +WHEEL_MAX_ANGLE
    if (current_angle > WHEEL_MAX_ANGLE) current_angle = WHEEL_MAX_ANGLE;
    if (current_angle < -WHEEL_MAX_ANGLE) current_angle = -WHEEL_MAX_ANGLE;

    // Directly set the wheel angle
    *wheel_angle = current_angle;

    // Calculate time difference
    uint32_t current_time = HAL_GetTick();  // In milliseconds
    float dt = (current_time - last_update_time) / 1000.0f;  // ms to seconds

    // Calculate angular velocity
    static float last_angle = 0.0f;
    float delta_angle = current_angle - last_angle;

    if (dt > 0.0001f) {  // Avoid division by zero
        *angular_velocity = delta_angle / dt;
    } else {
        *angular_velocity = 0.0f;
    }

    // Update for next loop
    last_angle = current_angle;
    last_update_time = current_time;

    // Map to steering value
    gSteering = map_wheel_position_to_axis(*wheel_angle);
}


void set_motor_pwm(float pwm_value, uint8_t direction) {
    // Assuming pwm_value ranges from 0 to 255
    uint32_t pulse = (uint32_t)((pwm_value / 255.0) * htim3.Init.Period);

    if (direction == 1) { // Forward
    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	} else if (direction == 0) { // Reverse
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
	} else { // Stop
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}

//void set_motor_direction(uint8_t direction) {
//    if (direction == 1) { // Forward
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // IN1 = HIGH
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // IN2 = LOW
//    } else if (direction == 0) { // Reverse
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // IN1 = LOW
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);   // IN2 = HIGH
//    } else { // Stop
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);   // IN1 = LOW
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);   // IN2 = LOW
//    }
//}

extern void signalTelemetryTask(uint8_t *Buf, uint32_t Len) {
	if (Len == sizeof(telemetry_packet))  // Verify the data size matches the struct size
	{
		// Process the received data (rx_buffer)
		memcpy(&telemetry_data, Buf, sizeof(telemetry_data));
		osSemaphoreRelease(uartMutexHandle);
	}
	else
	{
		int wkglwkgw = 1;
	}
}

void motor_rotate_left() {
    uint32_t target_position = gPosition - (MAX_REVOLUTIONS * ENCODER_RESOLUTION);
    while (gPosition > target_position) {
        // Set motor speed and direction
        //set_motor_direction(1);
        set_motor_pwm(100, 1);

        // Read Hall Sensor
        hall_voltage = gHall;
        if (hall_voltage > max_hall_voltage) {
            max_hall_voltage = hall_voltage;
            max_position = gPosition;
        }
    }

    // Stop Motor
    set_motor_pwm(0, 0);
}

void motor_rotate_right() {
    uint32_t target_position = gPosition + (MAX_REVOLUTIONS * ENCODER_RESOLUTION);
    while (gPosition < target_position) {
        // Set motor speed and direction
    	//set_motor_direction(0);
		set_motor_pwm(100, 0);

        // Read Hall Sensor
        hall_voltage = gHall;
        if (hall_voltage > max_hall_voltage) {
            max_hall_voltage = hall_voltage;
            max_position = gPosition;
        }
    }

    // Stop Motor
    set_motor_pwm(0, 0);
}

void read_hall_sensor() {
	if (adc_data_ready) {
	    adc_data_ready = 0;

	    gHall = adc_buffer[0];
	}
}

void Start_ADC_DMA() {
	HAL_TIM_Base_Start(&htim8);  // starts the timer
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adc_data_ready = 1;
    }
}

void move_to_position(uint32_t target_position) {
    while (gPosition != target_position) {
        if (gPosition < target_position) {
            // Rotate Right
        	//set_motor_direction(1);
			set_motor_pwm(100, 1);
        } else {
            // Rotate Left
        	//set_motor_direction(0);
			set_motor_pwm(100, 0);
        }
    }

    // Stop Motor
    set_motor_pwm(0, 0);
}

void processCAN() {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1) > 0) {
    	 if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK) {
            int32_t value = 0;

            // If the received message is for pedals, only use 2 bytes (int16_t)
            if (RxHeader.StdId >= 0x300 && RxHeader.StdId <= 0x302) {
                value = (int16_t)(
                    ((uint16_t)RxData[0]) |
                    ((uint16_t)RxData[1] << 8)  // Combine 2 bytes into int16_t
                );
            } else {
                // Steering Wheel Messages (4-byte int32_t)
                value = (int32_t)(
                    ((uint32_t)RxData[0]) |
                    ((uint32_t)RxData[1] << 8) |
                    ((uint32_t)RxData[2] << 16) |
                    ((uint32_t)RxData[3] << 24)
                );
            }

			switch (RxHeader.StdId) {
				// Steering Wheel
				case 0x200: user_input_data.buttons = (uint32_t)value; gDebugCounter1++; break;
				case 0x201: user_input_data.hall_analog_1 = (uint8_t)value; gDebugCounter1++; break;
				case 0x202: user_input_data.hall_analog_2 = (uint8_t)value; gDebugCounter1++; break;

				// Pedals
				case 0x300: pedal_data.encoder_1 = (int16_t)value; gDebugCounter2++; break;
				case 0x301: pedal_data.encoder_2 = (int16_t)value; gDebugCounter2++;  break;
				case 0x302: pedal_data.encoder_3 = (int16_t)value; gDebugCounter2++; break;
				default: break;
			}
		}
	}
}
/* USER CODE END Application */
