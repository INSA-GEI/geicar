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

#include "Services/uartdrv.h"
//#include "Services/i2cdrv.h"
#include "i2c_sensors.h"
#include "debug.h"
#include "com_usb.h"

/* Handle pour la file de messages */
QueueHandle_t APP_MessageQueue;

/* Prototypes */
void APP_MessageHandlerTask(void *pvParameters);
TaskHandle_t APP_MessageHandlerTaskhandle;

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

	/* Initialisation des autres sous-systemes */
	COM_USB_Init(&APP_MessageQueue);
	I2C_SensorsInit(&APP_MessageQueue);

	printf ("Done\n");
}

/**
 * @brief  Tâche de gestion des messages
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void APP_MessageHandlerTask(void *pvParameters) {
	void *receivedMessage;

	for (;;) {
		/* Attendre indéfiniment un message dans la file */
		if (xQueueReceive(APP_MessageQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
			// A reprendre
			printf("Message reçu : %s\n", (char *)receivedMessage);

			/* Libérer la mémoire du message après traitement */
			free(receivedMessage);
		}
	}
}








