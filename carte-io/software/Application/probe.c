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
static I2C_Sensor_ProbeResults_TypeDef i2c_results={0};

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
	Messages_TypeDef *msg;

	printf("[PROBE] Demarrage du scan des peripheriques\n");

	// Lance les "probe" de chaque capteurs et periph
	i2c_results = I2C_Sensors_Probe(); /* Recherche de périphériques I2C */

	msg= NEW_MESSAGE(MSG_ID_PROBE_RESULT, NULL);
	// i2c_results n'est pas alloué dynamiquement, ni allouée sur la stack qui
	// elle est allouée sur la heap. Les variables allouées sur les stack des taches
	// posent probleme avec free car elles leurs adresses sont bien dans l'intervalle
	// de la heap4 mais elles ne sont pas allouées dynamiquement
	msg->data = (uint8_t*)&i2c_results; // i2c_results n'est pas alloué dynamiquement, ni allouée sur la stack qui ell
	msg->length = sizeof(i2c_results);
//	msg->data = NULL;
//	msg->length = 0;

	// Envoie le message sur la queue de l'application
	if (xQueueSend(*ApplicationMessageQueue, (void* ) &msg,
			portMAX_DELAY) != pdPASS) {
		printf("[PROBE] Erreur d'envoi du message de probe\n");
	}

	printf("[PROBE] Fin du scan\n");

	vTaskDelete(NULL); // Supprime la tâche une fois le scan terminé
}
