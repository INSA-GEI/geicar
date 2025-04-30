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
#include "stm32u5xx_hal.h"

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
void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_I2C4_Init(void);
void MX_LPUART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_ADC1_Init(void);
void MX_LPTIM1_Init(void);
void MX_LPTIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_UART5_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_3_PHA_Pin GPIO_PIN_3
#define ENCODER_3_PHA_GPIO_Port GPIOE
#define ENCODER_3_PHB_Pin GPIO_PIN_4
#define ENCODER_3_PHB_GPIO_Port GPIOE
#define POWEROFF_Pin GPIO_PIN_5
#define POWEROFF_GPIO_Port GPIOE
#define COM_EXT_RX_Pin GPIO_PIN_0
#define COM_EXT_RX_GPIO_Port GPIOC
#define COM_EXT_TX_Pin GPIO_PIN_1
#define COM_EXT_TX_GPIO_Port GPIOC
#define ENCODER_1_PHB_Pin GPIO_PIN_2
#define ENCODER_1_PHB_GPIO_Port GPIOC
#define MOT_PWM1_Pin GPIO_PIN_0
#define MOT_PWM1_GPIO_Port GPIOA
#define MOT_PWM2_Pin GPIO_PIN_1
#define MOT_PWM2_GPIO_Port GPIOA
#define MOT_PWM3_Pin GPIO_PIN_2
#define MOT_PWM3_GPIO_Port GPIOA
#define MOT_PWM3A3_Pin GPIO_PIN_3
#define MOT_PWM3A3_GPIO_Port GPIOA
#define LED_ACTIVITY_Pin GPIO_PIN_4
#define LED_ACTIVITY_GPIO_Port GPIOA
#define PWR_CSLEEP_Pin GPIO_PIN_5
#define PWR_CSLEEP_GPIO_Port GPIOA
#define ADC_AN1_Pin GPIO_PIN_6
#define ADC_AN1_GPIO_Port GPIOA
#define ADC_AN2_Pin GPIO_PIN_7
#define ADC_AN2_GPIO_Port GPIOA
#define COM_GPS_TX_Pin GPIO_PIN_4
#define COM_GPS_TX_GPIO_Port GPIOC
#define COM_GPS_RX_Pin GPIO_PIN_5
#define COM_GPS_RX_GPIO_Port GPIOC
#define ENCODER_2_PHA_Pin GPIO_PIN_1
#define ENCODER_2_PHA_GPIO_Port GPIOB
#define ADC_VBAT_Pin GPIO_PIN_2
#define ADC_VBAT_GPIO_Port GPIOB
#define IO1_Pin GPIO_PIN_7
#define IO1_GPIO_Port GPIOE
#define IO2_Pin GPIO_PIN_8
#define IO2_GPIO_Port GPIOE
#define IO3_Pin GPIO_PIN_9
#define IO3_GPIO_Port GPIOE
#define IO4_Pin GPIO_PIN_10
#define IO4_GPIO_Port GPIOE
#define I2C_SCL_EXTERNAL_Pin GPIO_PIN_13
#define I2C_SCL_EXTERNAL_GPIO_Port GPIOB
#define I2C_SDA_EXTERNAL_Pin GPIO_PIN_14
#define I2C_SDA_EXTERNAL_GPIO_Port GPIOB
#define ENCODER_2_PHB_Pin GPIO_PIN_9
#define ENCODER_2_PHB_GPIO_Port GPIOD
#define I2C_SCL_ARBITRARY_Pin GPIO_PIN_12
#define I2C_SCL_ARBITRARY_GPIO_Port GPIOD
#define ENCODER_4_PHB_Pin GPIO_PIN_13
#define ENCODER_4_PHB_GPIO_Port GPIOD
#define SERVO_PWM1_Pin GPIO_PIN_6
#define SERVO_PWM1_GPIO_Port GPIOC
#define SERVO_PWM2_Pin GPIO_PIN_7
#define SERVO_PWM2_GPIO_Port GPIOC
#define SERVO_PWM3_Pin GPIO_PIN_8
#define SERVO_PWM3_GPIO_Port GPIOC
#define SERVO_PWM4_Pin GPIO_PIN_9
#define SERVO_PWM4_GPIO_Port GPIOC
#define COM_USB_TX_Pin GPIO_PIN_9
#define COM_USB_TX_GPIO_Port GPIOA
#define COM_USB_RX_Pin GPIO_PIN_10
#define COM_USB_RX_GPIO_Port GPIOA
#define COM_LIDAR1_TX_Pin GPIO_PIN_10
#define COM_LIDAR1_TX_GPIO_Port GPIOC
#define COM_LIDAR1_RX_Pin GPIO_PIN_11
#define COM_LIDAR1_RX_GPIO_Port GPIOC
#define COM_LIDAR2_TX_Pin GPIO_PIN_12
#define COM_LIDAR2_TX_GPIO_Port GPIOC
#define FDCAN_STBY_Pin GPIO_PIN_0
#define FDCAN_STBY_GPIO_Port GPIOD
#define COM_LIDAR2_RX_Pin GPIO_PIN_2
#define COM_LIDAR2_RX_GPIO_Port GPIOD
#define INT_MAGNETO_Pin GPIO_PIN_3
#define INT_MAGNETO_GPIO_Port GPIOD
#define INT_MAGNETO_EXTI_IRQn EXTI3_IRQn
#define INT_IMU_Pin GPIO_PIN_4
#define INT_IMU_GPIO_Port GPIOD
#define INT_IMU_EXTI_IRQn EXTI4_IRQn
#define INT_PRESSURE_Pin GPIO_PIN_5
#define INT_PRESSURE_GPIO_Port GPIOD
#define INT_PRESSURE_EXTI_IRQn EXTI5_IRQn
#define INT_LIGHT_Pin GPIO_PIN_6
#define INT_LIGHT_GPIO_Port GPIOD
#define INT_LIGHT_EXTI_IRQn EXTI6_IRQn
#define USER_BTN_Pin GPIO_PIN_7
#define USER_BTN_GPIO_Port GPIOD
#define USER_BTN_EXTI_IRQn EXTI7_IRQn
#define ENCODER_1_PHA_Pin GPIO_PIN_5
#define ENCODER_1_PHA_GPIO_Port GPIOB
#define ENCODER_4_PHA_Pin GPIO_PIN_6
#define ENCODER_4_PHA_GPIO_Port GPIOB
#define I2C_SDA_ARBITRARY_Pin GPIO_PIN_7
#define I2C_SDA_ARBITRARY_GPIO_Port GPIOB
#define I2C_SCL_INT_Pin GPIO_PIN_8
#define I2C_SCL_INT_GPIO_Port GPIOB
#define I2C_SDA_INT_Pin GPIO_PIN_9
#define I2C_SDA_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
