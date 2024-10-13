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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 150
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char receivedData[BUFFER_SIZE];
uint8_t recvIndex = 0;
uint8_t expectedLength = 0;  // Expected length of the command
uint8_t commandState = 0;    // State for command reception
// UART handle
extern UART_HandleTypeDef huart2;

int speed = 0;
int gear = 0;
int rpm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void process_command(char *command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_response(const char* response) {
	HAL_UART_Transmit(&huart2, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
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
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&receivedData[recvIndex], 1);  // Start receiving one byte
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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


void blink(uint32_t dur) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Replace GPIOA and GPIO_PIN_5 with the correct port and pin for LD2
	send_response("Light on\n");
	HAL_Delay(dur);                       // Wait for specified duration
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Turn off the LED
	send_response("Light off\n");
	HAL_Delay(dur);
}

void process_command(char* cmd) {
    char debug_message[50];  // Buffer to hold debug messages
    if (strncmp(cmd, "speedKmh:", 9) == 0) {
        int speedKmh = atoi(cmd + 9);  // Convert the string to an integer
        sprintf(debug_message, "Speed received: %d\n", speedKmh);
        send_response(debug_message);
    } else if (strncmp(cmd, "gear:", 5) == 0) {
        int gear = atoi(cmd + 5);  // Convert the string to an integer
        sprintf(debug_message, "Gear received: %d\n", gear);
        send_response(debug_message);
    } else if (strncmp(cmd, "rpm:", 4) == 0) {
        int rpm = atoi(cmd + 4);  // Convert the string to an integer
        sprintf(debug_message, "RPM received: %d\n", rpm);
        send_response(debug_message);
    } else {
        sprintf(debug_message, "Unknown command received: %s\n", cmd);
        send_response(debug_message);
    }
}


typedef enum {
    READ_LENGTH,
    READ_COMMAND,
} CommandState;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Check if the interrupt is for USART2
        uint8_t byte;
        HAL_UART_Receive_IT(huart, &byte, 1);  // Continue receiving bytes

        // Debugging output
        printf("Received byte: %c (%d), commandState: %d, recvIndex: %d\n", byte, byte, commandState, recvIndex);

        switch (commandState) {
            case READ_LENGTH:
                // We expect two bytes for the length of the command
                if (recvIndex < 2) {
                    receivedData[recvIndex++] = byte;  // Store the byte
                }

                // Check if we've received both bytes for length
                if (recvIndex == 2) {
                    // Convert the two ASCII characters to an integer length
                    uint8_t expectedLength = (receivedData[0] - '0') * 10 + (receivedData[1] - '0');
                    printf("Expected length: %d\n", expectedLength);  // Debugging output

                    if (expectedLength >= BUFFER_SIZE) {
                        printf("Error: Expected length %d exceeds buffer size\n", expectedLength);
                        recvIndex = 0;  // Reset for the next command
                        commandState = READ_LENGTH;  // Reset state
                    } else {
                        // Reset index for command data and switch state
                        recvIndex = 0;
                        commandState = READ_COMMAND;  // Move to command state
                    }
                }
                break;

            case READ_COMMAND:
                // Now receiving command data
                if (byte == '\n') {
                    // If a newline character is received, process the command
                    receivedData[recvIndex] = '\0';  // Null-terminate the string
                    printf("Full command received: %s\n", receivedData);  // Print for debugging

                    process_command(receivedData);  // Process the command

                    // Reset for the next command
                    recvIndex = 0;  // Reset index for next command
                    commandState = READ_LENGTH;  // Go back to length reading state

                    // Optionally clear the receivedData buffer
					memset(receivedData, 0, sizeof(receivedData));  // Clear the buffer
                } else {
                    // Regular command data
                    if (recvIndex < BUFFER_SIZE - 1) {
                        receivedData[recvIndex++] = byte;  // Store the byte
                    } else {
                        // Buffer overflow handling
                        printf("Error: Buffer overflow detected\n");
                        recvIndex = 0;  // Reset buffer
                        commandState = READ_LENGTH;  // Reset state
                    }
                }
                break;

            default:
                // Reset state if something goes wrong
                recvIndex = 0;
                commandState = READ_LENGTH;
                break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
            __HAL_UART_CLEAR_OREFLAG(huart);  // Clear overrun error flag
        }

        if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
            // Log the error only once to avoid flooding the output
            send_response("UART Error occurred\r\n");
        }

        // Re-enable UART receive interrupt after error handling
        HAL_UART_Receive_IT(&huart2, (uint8_t *)receivedData, BUFFER_SIZE);
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
