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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_Output_IN4_Pin GPIO_PIN_3
#define GPIO_Output_IN4_GPIO_Port GPIOA
#define GPIO_Output_IN3_Pin GPIO_PIN_7
#define GPIO_Output_IN3_GPIO_Port GPIOA
#define GPIO_Output_IN1_Pin GPIO_PIN_0
#define GPIO_Output_IN1_GPIO_Port GPIOB
#define GPIO_Output_IN2_Pin GPIO_PIN_1
#define GPIO_Output_IN2_GPIO_Port GPIOB
#define GPIO_Input_echo_Pin GPIO_PIN_10
#define GPIO_Input_echo_GPIO_Port GPIOA
#define GPIO_Input_IS5_Pin GPIO_PIN_11
#define GPIO_Input_IS5_GPIO_Port GPIOA
#define GPIO_Input_IS4_Pin GPIO_PIN_12
#define GPIO_Input_IS4_GPIO_Port GPIOA
#define GPIO_Input_IS1_Pin GPIO_PIN_3
#define GPIO_Input_IS1_GPIO_Port GPIOB
#define GPIO_Input_IS2_Pin GPIO_PIN_4
#define GPIO_Input_IS2_GPIO_Port GPIOB
#define GPIO_Input_IS3_Pin GPIO_PIN_5
#define GPIO_Input_IS3_GPIO_Port GPIOB
#define GPIO_Output_trig_Pin GPIO_PIN_7
#define GPIO_Output_trig_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
