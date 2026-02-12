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
#include "stm32c0xx_hal.h"

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
#define DISPLAY_CMD_DATA_Pin GPIO_PIN_14
#define DISPLAY_CMD_DATA_GPIO_Port GPIOC
#define DISPLAY_RESET_Pin GPIO_PIN_15
#define DISPLAY_RESET_GPIO_Port GPIOC
#define ON_OFF_BUTTON_Pin GPIO_PIN_0
#define ON_OFF_BUTTON_GPIO_Port GPIOA
#define OBC_SHUTDOWN_Pin GPIO_PIN_3
#define OBC_SHUTDOWN_GPIO_Port GPIOA
#define MASTER_PMOS_ENABLE_Pin GPIO_PIN_7
#define MASTER_PMOS_ENABLE_GPIO_Port GPIOA
#define SCREEN_PMOS_ENABLE_Pin GPIO_PIN_8
#define SCREEN_PMOS_ENABLE_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_11
#define RED_LED_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
