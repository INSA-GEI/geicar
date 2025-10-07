/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, US_TRIGGER_FRONT_LEFT_Pin|US_TRIGGER_FRONT_CENTER_Pin|US_TRIGGER_FRONT_RIGHT_Pin|US_TRIGGER_REAR_LEFT_Pin
                          |US_TRIGGER_REAR_CENTER_Pin|US_TRIGGER_REAR_RIGHT_Pin|ENABLE_MOTOR_LEFT_Pin|ENABLE_MOTOR_RIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PB_BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = PB_BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : US_TRIGGER_FRONT_LEFT_Pin US_TRIGGER_FRONT_CENTER_Pin US_TRIGGER_FRONT_RIGHT_Pin US_TRIGGER_REAR_LEFT_Pin
                           US_TRIGGER_REAR_CENTER_Pin US_TRIGGER_REAR_RIGHT_Pin ENABLE_MOTOR_LEFT_Pin ENABLE_MOTOR_RIGHT_Pin */
  GPIO_InitStruct.Pin = US_TRIGGER_FRONT_LEFT_Pin|US_TRIGGER_FRONT_CENTER_Pin|US_TRIGGER_FRONT_RIGHT_Pin|US_TRIGGER_REAR_LEFT_Pin
                          |US_TRIGGER_REAR_CENTER_Pin|US_TRIGGER_REAR_RIGHT_Pin|ENABLE_MOTOR_LEFT_Pin|ENABLE_MOTOR_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_ENABLE_Pin */
  GPIO_InitStruct.Pin = POWER_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB_STEERING_LEFT_Pin PB_STEERING_RIGHT_Pin */
  GPIO_InitStruct.Pin = PB_STEERING_LEFT_Pin|PB_STEERING_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : US_MEASURE_LEFT_Pin US_MEASURE_CENTER_Pin US_MEASURE_RIGHT_Pin */
  GPIO_InitStruct.Pin = US_MEASURE_LEFT_Pin|US_MEASURE_CENTER_Pin|US_MEASURE_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
