/*
 * i2c_sensors.c
 *
 *  Created on: May 19, 2025
 *      Author: dimercur
 */

#include "i2c_sensors.h"
#include "Services/i2cdrv.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
//#include "timers.h"

#include "config.h"
#include "messages.h"
#include "sw_timer.h"

#include <stdio.h>
#include <stdlib.h>

// Capteurs internes I2C
#include "Devices/internals_sensors.h"

/* Handle pour la file de messages */
QueueHandle_t I2C_Sensors_MessageQueue;
sw_timer_id_t I2C_Sensors_PeriodicTimer;          // Timer pour générer des évènements périodiques pour la scrutation des capteurs
TaskHandle_t I2C_Sensors_MessageHandlerTaskhandle;
static QueueHandle_t *ApplicationMessageQueue;    // Handle de la file de messages de l'application

void I2C_Sensors_MessagesHandlerTask(void *pvParameters);
//static void onTimerEvent(TimerHandle_t xTimer);
static void onTimerEvent(void *arg);

/**
 * @brief  Fonction d'initialisation des capteurs I2C
 * @retval None
 */
void I2C_SensorsInit(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue!=NULL);
	ApplicationMessageQueue = AppMsgQueue;

	printf ("[I2C SensorsInit] Initialisation... ");

	I2C_Sensors_MessageQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	if (I2C_Sensors_MessageQueue == NULL) {
		printf("[I2C SensorsInit] Erreur de création de la file\n");
		while (1);
	}
	vQueueAddToRegistry(I2C_Sensors_MessageQueue, "I2C_Sensors_MsgQ" );

	/* Initialisation de l'I2C1 - I2C_INTERNAL  */
	assert_param(I2C_Init(I2C_INTERNAL)==HAL_OK);

	/* Initialisation de l'I2C2 - I2C_EXTERNAL  */
	assert_param(I2C_Init(I2C_EXTERNAL)==HAL_OK);

	/* Initialisation de l'I2C4 - I2C_ARBITRARY  */
	assert_param(I2C_Init(I2C_ARBITRARY)==HAL_OK);

	/* Création de la tâche FreeRTOS de gestion des messages */
	xTaskCreate(I2C_Sensors_MessagesHandlerTask,
			"I2C_Sensors_MessagesHandlerTask",
			TASK_STACK_SIZE_STD,
			NULL,
			TASK_PRIO_I2C_SENSORS_MESSAGES_HANDLER_TASK,
			&I2C_Sensors_MessageHandlerTaskhandle);
	vTaskResume(I2C_Sensors_MessageHandlerTaskhandle);

	/* Création du timer pour générer périodiquement des évènements pour la scrutation des capteurs */
	I2C_Sensors_PeriodicTimer = SW_TIMER_Configure(10, onTimerEvent, NULL, SW_TIMER_PERIODIC);
	assert_param(I2C_Sensors_PeriodicTimer != SW_TIMER_NO_TIMER_AVAILABLE);

	// Démarrage du timer / lecture périodique
	assert_param(SW_TIMER_Start(I2C_Sensors_PeriodicTimer) == pdTRUE);

	printf ("Done\n");
}

/**
 * @brief  Retourne le handle de la file de messages I2C
 * @retval handle de la file de messages I2C
 */
QueueHandle_t* I2C_Sensors_GetMessageQueue(void) {
	return &I2C_Sensors_MessageQueue;
}

/**
 * @brief  Fonction pour lancer la recherche de périphériques I2C
 * @retval None
 */
I2C_Sensor_ProbeResults_TypeDef I2C_Sensors_Probe(void) {
	I2C_Sensor_ProbeResults_TypeDef results= {0};

	// Scan des peripheriques internes
	INT_SENSORS_Probe((Sensor_ProbeResults_TypeDef*)&(results.internalSensors));

	return results;
}

/**
 * @brief  Tâche de gestion des messages I2C
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void I2C_Sensors_MessagesHandlerTask(void *pvParameters) {
	Messages_TypeDef *receivedMessage;

	for (;;) {
		/* Attendre indéfiniment un message dans la file */
		if (xQueueReceive(I2C_Sensors_MessageQueue, (void*)&receivedMessage, portMAX_DELAY) == pdPASS) {
			// A reprendre
			//printf("Message ID recu : %u\n", (uint8_t)receivedMessage->id);

			/* Libérer la mémoire du message après traitement */
			DELETE_MESSAGE(receivedMessage);
		}
	}
}

/**
 * @brief  Timer périodique pour la gestion des capteurs I2C
 * @param  xTimer: Paramètres du timer(non utilisés ici)
 * @retval None
 */
//static void onTimerEvent(TimerHandle_t xTimer)
static void onTimerEvent(void *arg){
	static uint8_t counter =0;

	Messages_TypeDef *message = NEW_MESSAGE(MSG_ID_I2C_SENSORS_10MS_EVENT, NULL);

	// Envoi d'un message de scrutation des capteurs I2C à 10 ms
	// L'adresse du message est COPIÉE dans la file, pas de passage par ref
	if (xQueueSend(I2C_Sensors_MessageQueue, &message, portMAX_DELAY) != pdPASS) {
		printf("[I2C_Sensors] Échec de l'envoi du message\n");
		DELETE_MESSAGE(message);
	}

	counter++;
	if (counter > 5) {
		counter = 0;
		// Envoi d'un message de scrutation des capteurs I2C à 50 ms
		// L'adresse du precedent message a été COPIÉE dans la file, donc message est libre pour recevoir une autre allocation
		message = NEW_MESSAGE(MSG_ID_I2C_SENSORS_50MS_EVENT, NULL);
		if (xQueueSend(I2C_Sensors_MessageQueue, &message,
				portMAX_DELAY) != pdPASS) {
			printf("[I2C_Sensors] Échec de l'envoi du message\n");
			DELETE_MESSAGE(message);
		}
	}
}

