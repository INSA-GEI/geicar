/*
 * app.c
 *
 *  Created on: Dec 17, 2024
 *      Author: dimercur
 */
#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "messages.h"
#include "sw_timer.h"

#include "Services/uartdrv.h"
#include "i2c_sensors.h"
#include "debug.h"
#include "com_usb.h"
#include "motors_servos.h"
#include "gpio.h"

#include "probe.h"

/* Handle pour la file de messages */
QueueHandle_t APP_MessageQueue;

/* Prototypes */
void APP_MessageHandlerTask(void *pvParameters);
TaskHandle_t APP_MessageHandlerTaskhandle;
void APP_SendVersion(void);

/**
 * @brief  Fonction d'initialisation de l'application
 * @retval None
 */
void APP_Init(void) {
	printf ("[APP_Init] Initialisation... ");

	APP_MessageQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	if (APP_MessageQueue == NULL) {
		printf("[APP Init] Erreur de création de la file\n");
		while (1);
	}
	vQueueAddToRegistry(APP_MessageQueue, "APP_MsgQ" );

	/* Créer les tâches */
	xTaskCreate(APP_MessageHandlerTask,
			"APP_MsgHandler",
			TASK_STACK_SIZE_APPLICATION,
			NULL,
			TASK_PRIO_APP_MSG_HANDLER,
			&APP_MessageHandlerTaskhandle);
	vTaskResume(APP_MessageHandlerTaskhandle);

	/* Initialisation du support de debug */
	DEBUG_Init();

	/* Initialisation du timer logiciel */
	if (SW_TIMER_Init() != pdTRUE) {
		printf("[APP Init] Erreur d'initialisation du service de timers\n");
		while (1)
			; // Erreur, on boucle
	}

	/* Initialisation des autres sous-systemes */
	COM_USB_Init(&APP_MessageQueue);
	I2C_SensorsInit(&APP_MessageQueue);
	GPIO_Init(&APP_MessageQueue);
	MOTORS_SERVOS_Init();

	APP_SendVersion();

	/* Recherche de périphériques */
	PROBE_Init(&APP_MessageQueue);

	printf ("Done\n");
}

/**
 * @brief  Tâche de gestion des messages
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void APP_MessageHandlerTask(void *pvParameters) {
	Messages_TypeDef *msg;

	for (;;) {
		/* Attendre indéfiniment un message dans la file */
		if (xQueueReceive(APP_MessageQueue, (void*)&msg, portMAX_DELAY) == pdPASS) {

			printf("[APP]Message recu, ID : %d\n", msg->id);

			switch (msg->id) {
			case MSG_ID_GPIO_CONFIGURE:
			case MSG_ID_GPIO_SET_STATE:
			case MSG_ID_GPIO_GET_STATE:
				GPIO_MessageProcessor(msg);
				break;
			case MSG_ID_MOTORS_CONFIGURE:
			case MSG_ID_SERVOS_CONFIGURE:
			case MSG_ID_MOTORS_SET_SPEED:
			case MSG_ID_SERVOS_SET_POSITION:
				MOTORS_SERVOS_MessageProcessor(msg);
				break;
			case MSG_ID_PROBE_RESULT:
				/* Traiter le message de résultat de probe */
				printf("[APP] Probe result received: %d\n", msg->length);
				break;
			default:
				printf("[APP] Message ID invalide");
				break;
			}

			/* Libérer la mémoire du message après traitement */
			//DELETE_MESSAGE(msg);
			free(msg->data); // Libération de la mémoire allouée pour les données
			free(msg); // Libération de la mémoire allouée pour le message

			printf("[APP] Message libere\n");
		}
	}
}
/**
 * @brief  Envoie la version de l'application sur le port USB
 * @retval None
 */
void APP_SendVersion(void) {
	Messages_TypeDef msg;

	printf("[APP] Version %s\n", VERSION_STRING);

	msg.id = MSG_ID_VERSION;
	msg.length = 2; // 2 octets, 1 pour le major et 1 pour le minor
	uint8_t verData[msg.length];
	verData[0] = MAJOR_VER;
	verData[1] = MINOR_VER;

	msg.data = (uint8_t*)verData;

	COM_USB_SendData(&msg);
	// Pas de libération de mémoire ici, le buffer est alloué sur la stack
}


