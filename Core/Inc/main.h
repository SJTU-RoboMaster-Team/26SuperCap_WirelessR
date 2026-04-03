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
#include "stm32g4xx_hal.h"

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
#define DEAD_TIME 80
#define PWM_PERIOD 27200
#define EN_4201_Pin GPIO_PIN_13
#define EN_4201_GPIO_Port GPIOC
#define BUZZ_Pin GPIO_PIN_0
#define BUZZ_GPIO_Port GPIOA
#define IN_V_Pin GPIO_PIN_0
#define IN_V_GPIO_Port GPIOB
#define OUT_C_Pin GPIO_PIN_1
#define OUT_C_GPIO_Port GPIOB
#define IN_C_Pin GPIO_PIN_2
#define IN_C_GPIO_Port GPIOB
#define CAP_V_Pin GPIO_PIN_15
#define CAP_V_GPIO_Port GPIOB
#define ASK_Pin GPIO_PIN_8
#define ASK_GPIO_Port GPIOA
#define CAP_C_Pin GPIO_PIN_9
#define CAP_C_GPIO_Port GPIOA
#define LED_PIN_Pin GPIO_PIN_15
#define LED_PIN_GPIO_Port GPIOA
#define SCL_4201_Pin GPIO_PIN_8
#define SCL_4201_GPIO_Port GPIOB
#define SDA_4201_Pin GPIO_PIN_9
#define SDA_4201_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
