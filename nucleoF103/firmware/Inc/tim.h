/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN Private defines */
// Timer & Channel definitions
#define TIM_MOTOR_LEFT TIM1
#define TIM_MOTOR_RIGHT TIM1
#define TIM_STEERING TIM4
#define TIM_ENCODER_LEFT TIM2
#define TIM_ENCODER_RIGHT TIM2
#define TIM_US TIM3
#define TIM_MOTOR_LEFT_CHANNEL TIM_CHANNEL_1
#define TIM_MOTOR_RIGHT_CHANNEL TIM_CHANNEL_2
#define TIM_STEERING_CHANNEL TIM_CHANNEL_1
#define TIM_ENCODER_LEFT_CHANNEL TIM_CHANNEL_3
#define TIM_ENCODER_RIGHT_CHANNEL TIM_CHANNEL_1
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

