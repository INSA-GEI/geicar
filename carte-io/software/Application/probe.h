/*
 * probe.h
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */

#ifndef PROBE_H_
#define PROBE_H_

#include "FreeRTOS.h"
#include "queue.h"

void PROBE_Init(QueueHandle_t *AppMsgQueue);
BaseType_t PROBE_Start(void);

#endif /* PROBE_H_ */
