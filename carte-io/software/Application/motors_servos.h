/*
 * motors_servos.h
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#ifndef MOTORS_SERVOS_H_
#define MOTORS_SERVOS_H_

#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

void MOTORS_SERVOS_Init(QueueHandle_t *AppMsgQueue);

#endif /* MOTORS_SERVOS_H_ */
