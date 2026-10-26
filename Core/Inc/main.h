/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define CAN2_RX_Pin GPIO_PIN_5
#define CAN2_RX_GPIO_Port GPIOB
#define USART6_TX_Pin GPIO_PIN_14
#define USART6_TX_GPIO_Port GPIOG
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
#define CAN2_TX_Pin GPIO_PIN_6
#define CAN2_TX_GPIO_Port GPIOB
#define CAN1_RX_Pin GPIO_PIN_0
#define CAN1_RX_GPIO_Port GPIOD
#define USART3_RX_Pin GPIO_PIN_11
#define USART3_RX_GPIO_Port GPIOC
#define USART3_TX_Pin GPIO_PIN_10
#define USART3_TX_GPIO_Port GPIOC
#define USART6_RX_Pin GPIO_PIN_9
#define USART6_RX_GPIO_Port GPIOG
#define CAN1_TX_Pin GPIO_PIN_1
#define CAN1_TX_GPIO_Port GPIOD
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define ADC_BAT_Pin GPIO_PIN_10
#define ADC_BAT_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define Buzzer_Pin GPIO_PIN_14
#define Buzzer_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
