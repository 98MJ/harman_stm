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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_3
#define LCD_RST_GPIO_Port GPIOC
#define STEP_1_Pin GPIO_PIN_1
#define STEP_1_GPIO_Port GPIOA
#define STEP_2_Pin GPIO_PIN_4
#define STEP_2_GPIO_Port GPIOA
#define STEP_3_Pin GPIO_PIN_0
#define STEP_3_GPIO_Port GPIOB
#define RESTART_btn_Pin GPIO_PIN_14
#define RESTART_btn_GPIO_Port GPIOB
#define RESTART_btn_EXTI_IRQn EXTI15_10_IRQn
#define EM_btn_Pin GPIO_PIN_15
#define EM_btn_GPIO_Port GPIOB
#define EM_btn_EXTI_IRQn EXTI15_10_IRQn
#define buzzer_Pin GPIO_PIN_6
#define buzzer_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_8
#define IN2_GPIO_Port GPIOC
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOC
#define echo_Pin GPIO_PIN_8
#define echo_GPIO_Port GPIOA
#define trigger_Pin GPIO_PIN_9
#define trigger_GPIO_Port GPIOA
#define TEST_GREEN_Pin GPIO_PIN_10
#define TEST_GREEN_GPIO_Port GPIOC
#define TEST_RED_Pin GPIO_PIN_11
#define TEST_RED_GPIO_Port GPIOC
#define TEST_BLUE_Pin GPIO_PIN_12
#define TEST_BLUE_GPIO_Port GPIOC
#define servo_Pin GPIO_PIN_6
#define servo_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LCD_WIDTH 320
#define LCD_HEIGHT 240
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */