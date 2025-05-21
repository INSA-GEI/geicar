/*
 * motors_servos.c
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#include "motors_servos.h"
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "messages.h"

static QueueHandle_t *ApplicationMessageQueue;    // Handle de la file de messages de l'application
QueueHandle_t MOTORS_SERVOS_MessageQueue;

TaskHandle_t MOTORS_SERVOS_MessageHandlerTaskhandle;
void MOTORS_SERVOS_MessagesHandlerTask(void *pvParameters);

extern void MX_TIM5_Init(void);
extern void MX_TIM8_Init(void);

void MOTORS_SERVOS_Init(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue!=NULL);
	ApplicationMessageQueue = AppMsgQueue;

	printf ("[MOTORS SENSORS] Initialisation... ");

	/* Initialisation du timer TIM5 (moteurs) */
	MX_TIM5_Init();

	/* Initialisation du timer TIM8 (servos) */
	MX_TIM8_Init();

	MOTORS_SERVOS_MessageQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	if (MOTORS_SERVOS_MessageQueue == NULL) {
		printf("[MOTORS SERVOS] Erreur de création de la file\n");
		while (1);
	}
	vQueueAddToRegistry(MOTORS_SERVOS_MessageQueue, "MOTORS_SERVOS_MsgQ" );

	/* Création de la tâche FreeRTOS de gestion des messages */
	xTaskCreate(MOTORS_SERVOS_MessagesHandlerTask,
				"MOTORS_SERVOS_MessagesHandlerTask",
				TASK_STACK_SIZE_STD,
				NULL,
				TASK_PRIO_MOTORS_SERVOS_MESSAGES_HANDLER_TASK,
				&MOTORS_SERVOS_MessageHandlerTaskhandle);
		vTaskResume(MOTORS_SERVOS_MessageHandlerTaskhandle);

	printf ("Done\n");
}

/**
 * @brief  Tâche de gestion des messages moteurs/servos
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void MOTORS_SERVOS_MessagesHandlerTask(void *pvParameters) {
	Messages_TypeDef *receivedMessage;

	for (;;) {
		/* Attendre indéfiniment un message dans la file */
		if (xQueueReceive(MOTORS_SERVOS_MessageQueue, (void*) &receivedMessage,
				portMAX_DELAY) == pdPASS) {
			// A reprendre
			printf("Message ID reçu : %u\n", (uint8_t) receivedMessage->id);

			/* Libérer la mémoire du message après traitement */
			DELETE_MESSAGE(receivedMessage);
		}
	}
}
