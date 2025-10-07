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
#define PB_BLUE_BUTTON_Pin GPIO_PIN_13
#define PB_BLUE_BUTTON_GPIO_Port GPIOC
#define PB_BLUE_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define ADC1_10_I_STEERING_Pin GPIO_PIN_0
#define ADC1_10_I_STEERING_GPIO_Port GPIOC
#define US_TRIGGER_FRONT_LEFT_Pin GPIO_PIN_1
#define US_TRIGGER_FRONT_LEFT_GPIO_Port GPIOC
#define US_TRIGGER_FRONT_CENTER_Pin GPIO_PIN_2
#define US_TRIGGER_FRONT_CENTER_GPIO_Port GPIOC
#define US_TRIGGER_FRONT_RIGHT_Pin GPIO_PIN_3
#define US_TRIGGER_FRONT_RIGHT_GPIO_Port GPIOC
#define ADC1_0_VBAT_Pin GPIO_PIN_0
#define ADC1_0_VBAT_GPIO_Port GPIOA
#define ADC1_1_STEERING_ANGLE_Pin GPIO_PIN_1
#define ADC1_1_STEERING_ANGLE_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ADC1_4_I_MOTOR_LEFT_Pin GPIO_PIN_4
#define ADC1_4_I_MOTOR_LEFT_GPIO_Port GPIOA
#define ADC1_5_I_MOTOR_RIGHT_Pin GPIO_PIN_5
#define ADC1_5_I_MOTOR_RIGHT_GPIO_Port GPIOA
#define POWER_ENABLE_Pin GPIO_PIN_6
#define POWER_ENABLE_GPIO_Port GPIOA
#define PWM_COMP_LEFT_Pin GPIO_PIN_7
#define PWM_COMP_LEFT_GPIO_Port GPIOA
#define US_TRIGGER_REAR_LEFT_Pin GPIO_PIN_4
#define US_TRIGGER_REAR_LEFT_GPIO_Port GPIOC
#define US_TRIGGER_REAR_CENTER_Pin GPIO_PIN_5
#define US_TRIGGER_REAR_CENTER_GPIO_Port GPIOC
#define PWM_COMP_RIGHT_Pin GPIO_PIN_0
#define PWM_COMP_RIGHT_GPIO_Port GPIOB
#define IC_ENCODER_MOTOR_LEFT_Pin GPIO_PIN_10
#define IC_ENCODER_MOTOR_LEFT_GPIO_Port GPIOB
#define PB_STEERING_LEFT_Pin GPIO_PIN_14
#define PB_STEERING_LEFT_GPIO_Port GPIOB
#define PB_STEERING_RIGHT_Pin GPIO_PIN_15
#define PB_STEERING_RIGHT_GPIO_Port GPIOB
#define US_MEASURE_LEFT_Pin GPIO_PIN_6
#define US_MEASURE_LEFT_GPIO_Port GPIOC
#define US_MEASURE_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define US_TRIGGER_REAR_RIGHT_Pin GPIO_PIN_7
#define US_TRIGGER_REAR_RIGHT_GPIO_Port GPIOC
#define US_MEASURE_CENTER_Pin GPIO_PIN_8
#define US_MEASURE_CENTER_GPIO_Port GPIOC
#define US_MEASURE_CENTER_EXTI_IRQn EXTI9_5_IRQn
#define US_MEASURE_RIGHT_Pin GPIO_PIN_9
#define US_MEASURE_RIGHT_GPIO_Port GPIOC
#define US_MEASURE_RIGHT_EXTI_IRQn EXTI9_5_IRQn
#define PWM_MOTOR_LEFT_Pin GPIO_PIN_8
#define PWM_MOTOR_LEFT_GPIO_Port GPIOA
#define PWM_MOTOR_RIGHT_Pin GPIO_PIN_9
#define PWM_MOTOR_RIGHT_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define IC_ENCODER_MOTOR_RIGHT_Pin GPIO_PIN_15
#define IC_ENCODER_MOTOR_RIGHT_GPIO_Port GPIOA
#define ENABLE_MOTOR_LEFT_Pin GPIO_PIN_10
#define ENABLE_MOTOR_LEFT_GPIO_Port GPIOC
#define ENABLE_MOTOR_RIGHT_Pin GPIO_PIN_11
#define ENABLE_MOTOR_RIGHT_GPIO_Port GPIOC
#define PWM_SERVO_STEERING_Pin GPIO_PIN_6
#define PWM_SERVO_STEERING_GPIO_Port GPIOB
#define PWM_SERVO_SPARE_Pin GPIO_PIN_7
#define PWM_SERVO_SPARE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
