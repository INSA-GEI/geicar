/*
 * tasks.h
 *
 *  Created on: Aug 27, 2025
 *      Author: dimercur
 */

#ifndef TASKS_H_
#define TASKS_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Handle vers la queue de la tache TASK_AppLoop */
extern QueueHandle_t xAppLoopQueue;

void TASKS_Init(void);

#endif /* TASKS_H_ */
