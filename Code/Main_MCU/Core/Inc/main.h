/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define CONNECT_DET_3_Pin GPIO_PIN_0
#define CONNECT_DET_3_GPIO_Port GPIOC
#define WORKING_LED_3_Pin GPIO_PIN_1
#define WORKING_LED_3_GPIO_Port GPIOC
#define CONNECT_LED_3_Pin GPIO_PIN_3
#define CONNECT_LED_3_GPIO_Port GPIOC
#define WORKING_LED_1_Pin GPIO_PIN_4
#define WORKING_LED_1_GPIO_Port GPIOA
#define CONNECT_LED_1_Pin GPIO_PIN_5
#define CONNECT_LED_1_GPIO_Port GPIOA
#define CONNECT_DET_1_Pin GPIO_PIN_6
#define CONNECT_DET_1_GPIO_Port GPIOA
#define WORKING_LED_2_Pin GPIO_PIN_7
#define WORKING_LED_2_GPIO_Port GPIOA
#define CONNECT_DET_2_Pin GPIO_PIN_5
#define CONNECT_DET_2_GPIO_Port GPIOC
#define CONNECT_LED_2_Pin GPIO_PIN_0
#define CONNECT_LED_2_GPIO_Port GPIOB
#define BRAKE_2_Pin GPIO_PIN_9
#define BRAKE_2_GPIO_Port GPIOC
#define WORKING_LED_4_Pin GPIO_PIN_9
#define WORKING_LED_4_GPIO_Port GPIOA
#define CONNECT_LED_4_Pin GPIO_PIN_10
#define CONNECT_LED_4_GPIO_Port GPIOA
#define CONNECT_DET_4_Pin GPIO_PIN_10
#define CONNECT_DET_4_GPIO_Port GPIOC
#define SPEED_1_Pin GPIO_PIN_11
#define SPEED_1_GPIO_Port GPIOC
#define SPEED_2_Pin GPIO_PIN_12
#define SPEED_2_GPIO_Port GPIOC
#define START_BUTTON_Pin GPIO_PIN_2
#define START_BUTTON_GPIO_Port GPIOD
#define Brake1_Pin GPIO_PIN_5
#define Brake1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define PI 3.1415926535897932384626433832795028
#define NORMAL_WORKING 0xAD
#define WRONG_SPEED 0xAE
#define BUFFER_LENGTH 1000
#define LIDAR_SPEED 1350
#define POINTS_THRESHOLD 3

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void setup();
void loop();

void reset_uarts();
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
