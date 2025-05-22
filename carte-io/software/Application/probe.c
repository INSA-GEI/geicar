/*
 * probe.c
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */
#include "stm32u5xx_hal.h"
#include "probe.h"

#include "config.h"

#include <stdio.h>
#include <stdlib.h>

#include "i2c_sensors.h"
#include "messages.h"

void PROBE_Task(void *pvParameters);
TaskHandle_t PROBE_Taskhandle;
static QueueHandle_t *ApplicationMessageQueue;    // Handle de la file de messages de l'application

void PROBE_Init(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue!=NULL);
	ApplicationMessageQueue = AppMsgQueue;

	printf ("[PROBE] Initialisation... ");

	/* Création de la tâche FreeRTOS */
	xTaskCreate(PROBE_Task,
			"PROBE_Task",
			TASK_STACK_SIZE_STD,
			NULL,
			TASK_PRIO_PROBE_RUN,
			&PROBE_Taskhandle);
	vTaskResume(PROBE_Taskhandle);

	printf ("Done\n");
}

/* Fonction de la tâche qui traite les messages */
void PROBE_Task(void *pvParameters) {
	printf("[PROBE] Demarrage du scan des peripheriques\n");

	// Lance les "probe" de chaque capteurs et periph
	I2C_Sensors_Probe(); /* Recherche de périphériques I2C */

	printf("[PROBE] Fin du scan\n");
}
