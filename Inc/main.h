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
#include "stm32f1xx_hal.h"

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
#define SW5_Pin GPIO_PIN_13
#define SW5_GPIO_Port GPIOC
#define SW5_EXTI_IRQn EXTI15_10_IRQn
#define SW4_Pin GPIO_PIN_14
#define SW4_GPIO_Port GPIOC
#define SW4_EXTI_IRQn EXTI15_10_IRQn
#define SEG_DP_Pin GPIO_PIN_15
#define SEG_DP_GPIO_Port GPIOC
#define PIN18_Pin GPIO_PIN_0
#define PIN18_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOD
#define SW3_EXTI_IRQn EXTI1_IRQn
#define PIN1_Pin GPIO_PIN_0
#define PIN1_GPIO_Port GPIOA
#define PIN3_Pin GPIO_PIN_1
#define PIN3_GPIO_Port GPIOA
#define PIN4_Pin GPIO_PIN_2
#define PIN4_GPIO_Port GPIOA
#define PIN6_Pin GPIO_PIN_3
#define PIN6_GPIO_Port GPIOA
#define PIN7_Pin GPIO_PIN_4
#define PIN7_GPIO_Port GPIOA
#define PIN9_Pin GPIO_PIN_5
#define PIN9_GPIO_Port GPIOA
#define PIN10_Pin GPIO_PIN_6
#define PIN10_GPIO_Port GPIOA
#define PIN12_Pin GPIO_PIN_7
#define PIN12_GPIO_Port GPIOA
#define CATH1_Pin GPIO_PIN_0
#define CATH1_GPIO_Port GPIOB
#define CATH2_Pin GPIO_PIN_1
#define CATH2_GPIO_Port GPIOB
#define CATH3_Pin GPIO_PIN_2
#define CATH3_GPIO_Port GPIOB
#define SEG_B_Pin GPIO_PIN_10
#define SEG_B_GPIO_Port GPIOB
#define SEG_A_Pin GPIO_PIN_11
#define SEG_A_GPIO_Port GPIOB
#define PIN17_Pin GPIO_PIN_12
#define PIN17_GPIO_Port GPIOB
#define PIN11_Pin GPIO_PIN_13
#define PIN11_GPIO_Port GPIOB
#define PIN8_Pin GPIO_PIN_14
#define PIN8_GPIO_Port GPIOB
#define PIN5_Pin GPIO_PIN_15
#define PIN5_GPIO_Port GPIOB
#define PIN13_Pin GPIO_PIN_8
#define PIN13_GPIO_Port GPIOA
#define PIN14_Pin GPIO_PIN_9
#define PIN14_GPIO_Port GPIOA
#define PIN19_Pin GPIO_PIN_10
#define PIN19_GPIO_Port GPIOA
#define PIN15_Pin GPIO_PIN_11
#define PIN15_GPIO_Port GPIOA
#define PIN16_Pin GPIO_PIN_12
#define PIN16_GPIO_Port GPIOA
#define PIN2_Pin GPIO_PIN_15
#define PIN2_GPIO_Port GPIOA
#define CATH4_Pin GPIO_PIN_4
#define CATH4_GPIO_Port GPIOB
#define SEG_G_Pin GPIO_PIN_5
#define SEG_G_GPIO_Port GPIOB
#define SEG_F_Pin GPIO_PIN_6
#define SEG_F_GPIO_Port GPIOB
#define SEG_E_Pin GPIO_PIN_7
#define SEG_E_GPIO_Port GPIOB
#define SEG_D_Pin GPIO_PIN_8
#define SEG_D_GPIO_Port GPIOB
#define SEG_C_Pin GPIO_PIN_9
#define SEG_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define HDMI_PINS 19
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
