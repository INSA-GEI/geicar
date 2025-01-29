/*
 * probe.c
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */
#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"

void PROBE_RunTask(void *pvParameters);
TaskHandle_t PROBE_RunTaskhandle;

void PROBE_Init(void) {
	/* Création de la tâche FreeRTOS */
	xTaskCreate(PROBE_RunTask,
			"PROBE_Run",
			TASK_STACK_SIZE_STD,
			NULL,
			TASK_PRIO_PROBE_RUN,
			&PROBE_RunTaskhandle);
	vTaskResume(PROBE_RunTaskhandle);
}

/* Fonction de la tâche qui traite les messages */
void PROBE_RunTask(void *pvParameters) {
	// Lance les "probe" de chaque capteurs et periph

}
