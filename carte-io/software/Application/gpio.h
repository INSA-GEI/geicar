/*
 * gpio.h
 *
 *  Created on: May 22, 2025
 *      Author: dimercur
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

void GPIO_Init(QueueHandle_t *AppMsgQueue);

#endif /* GPIO_H_ */
