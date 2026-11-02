/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define REED_BOTTOM_Pin GPIO_PIN_4
#define REED_BOTTOM_GPIO_Port GPIOA
#define REED_TOP_Pin GPIO_PIN_5
#define REED_TOP_GPIO_Port GPIOA
#define LIMIT_CYL_Pin GPIO_PIN_6
#define LIMIT_CYL_GPIO_Port GPIOA
#define LIMIT_X_Pin GPIO_PIN_0
#define LIMIT_X_GPIO_Port GPIOB
#define LIMIT_X_EXTI_IRQn EXTI0_IRQn
#define LIMIT_Y_Pin GPIO_PIN_1
#define LIMIT_Y_GPIO_Port GPIOB
#define LIMIT_Y_EXTI_IRQn EXTI1_IRQn
#define LIMIT_Z_Pin GPIO_PIN_10
#define LIMIT_Z_GPIO_Port GPIOB
#define LIMIT_Z_EXTI_IRQn EXTI15_10_IRQn
#define DIR_X_Pin GPIO_PIN_12
#define DIR_X_GPIO_Port GPIOB
#define STEP_X_Pin GPIO_PIN_13
#define STEP_X_GPIO_Port GPIOB
#define DIR_Y_Pin GPIO_PIN_14
#define DIR_Y_GPIO_Port GPIOB
#define STEP_Y_Pin GPIO_PIN_15
#define STEP_Y_GPIO_Port GPIOB
#define RELAY_UP_Pin GPIO_PIN_11
#define RELAY_UP_GPIO_Port GPIOA
#define RELAY_DOWN_Pin GPIO_PIN_12
#define RELAY_DOWN_GPIO_Port GPIOA
#define DIR_Z_Pin GPIO_PIN_3
#define DIR_Z_GPIO_Port GPIOB
#define STEP_Z_Pin GPIO_PIN_4
#define STEP_Z_GPIO_Port GPIOB
#define ENABLE_PIN_Pin GPIO_PIN_5
#define ENABLE_PIN_GPIO_Port GPIOB
#define WELD_PIN_Pin GPIO_PIN_6
#define WELD_PIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
