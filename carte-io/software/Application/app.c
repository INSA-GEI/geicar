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
#include "leds.h"

#include "probe.h"

/* Handle pour la file de messages */
QueueHandle_t APP_MessageQueue;

/* Prototypes */
void APP_MessageHandlerTask(void *pvParameters);
TaskHandle_t APP_MessageHandlerTaskhandle;
void APP_SendVersion(void);
BaseType_t APP_SendError(Messages_ErrorTypeDef errorType);

APP_MachineState_TypeDef APP_MachineState = { .state = APP_STATE_INIT,
		.batteryVoltage = 0.0f, .gpsFix = APP_GPS_NOT_FIX, };

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

	/* Initialisation des LEDs */
	LEDS_Init();

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

	/* Changement d'état vers l'état de probe */
	if (APP_ChangeState(APP_STATE_PROBE) != pdTRUE) {
		printf("[APP Init] Erreur de changement d'état vers l'état de probe\n");
		while (1)
			; // Erreur, on boucle
	}

	printf ("Done\n");
}

/**
 * @brief  Tâche de gestion des messages
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void APP_MessageHandlerTask(void *pvParameters) {
	Messages_TypeDef *msg;
	Messages_TypeDef ansMsg;

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
				if (APP_MachineState.state != APP_STATE_INIT
						&& APP_MachineState.state != APP_STATE_PROBE
						&& APP_MachineState.state != APP_STATE_ERROR) {
					MOTORS_SERVOS_MessageProcessor(msg); // Pas de configuration de moteurs/servos dans les etats autres que RUNNING et LOW BAT
				}

				break;
			case MSG_ID_PROBE_RESULT:
				/* Traiter le message de résultat de probe */
				printf("[APP] Probe result received: %d\n", msg->length);
				ansMsg.id = MSG_ID_PROBE_RESULT;
				ansMsg.length = msg->length; // Longueur des données du message
				ansMsg.data = msg->data; // Les données sont allouées par l'emetteur, on ne recopie que la ref

				// COM_USB_SendData() recopie les données dans un buffer interne : ansMsg peut donc etre alloué sur
				// la stack : a la fin de la fonction, le buffer est libéré automatiquement mais ça ne perturbe pas
				// l'envoi des données par COM_USB_SendData
				// idem avec msg->data qui est alloué dynamiquement mais sera recopié dans un buffer interne par
				// COM_USB_SendData. On peut donc le liberer à la fin du switch
				COM_USB_SendData(&ansMsg);
				// Pas de libération de mémoire ici, le buffer est alloué sur la stack

				/* Changement d'état vers l'état de running */
				if (APP_ChangeState(APP_STATE_RUNNING) != pdTRUE) {
					printf("[APP Init] Erreur de changement d'état vers l'état running\n");
					while (1)
						; // Erreur, on boucle
				}

				break;
			case MSG_ID_ERROR:
				/* Traiter le message d'erreur */
				printf("[APP] Error message received\n");
				APP_SendError(*(Messages_ErrorTypeDef*)msg->data);

				break;
			default:
				printf("[APP] Message ID invalide");
				APP_SendError(MSG_ERROR_INVALID_ID);
				break;
			}

			/* Libérer la mémoire du message après traitement */
			DELETE_MESSAGE(msg);

			printf("[APP] Message libere\n");
		}
	}
}

/**
 * @brief  Change l'état de la machine d'état de l'application
 * @param  newState: Nouvel état à atteindre
 * @retval pdTRUE si le changement d'état a réussi, pdFALSE sinon
 */
BaseType_t APP_ChangeState(APP_State_EnumTypeDef newState) {
	assert_param(newState >= APP_STATE_INIT && newState <= APP_STATE_SHUTDOWN);

	if (APP_MachineState.state != newState) {
		// Changement d'état
		printf("[APP] Changement d'état de %d vers %d\n", APP_MachineState.state,
				newState);
		LEDS_SetActivityState(newState); // Mettre à jour l'état de la LED d'activité

		switch (newState) {
		case APP_STATE_INIT:
			// Initialisation, rien à faire ici
			break;
		case APP_STATE_PROBE:
			if (APP_MachineState.state == APP_STATE_INIT) {
				PROBE_Start(); // Démarrer la tâche de probe
			} else
				newState = APP_MachineState.state; // On ne change pas l'état si on n'est pas dans l'état INIT
			break;
		case APP_STATE_RUNNING:
			// Démarrer les tâches principales
			break;
		case APP_STATE_LOW_BATTERY:
			printf("[APP] Alerte batterie faible !\n");
			break;
		case APP_STATE_ERROR:
			printf("[APP] Erreur détectée !\n");
			break;
		case APP_STATE_SHUTDOWN:
			printf("[APP] Arrêt du système...\n");
			break;
		default:
			printf("[APP] État inconnu : %d\n", newState);
			return pdFALSE; // État inconnu, on ne change pas l'état
		}

		APP_MachineState.state = newState;

		return pdTRUE; // Changement d'état réussi
	} else
		return pdFALSE; // Pas de changement d'état, on retourne faux
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

BaseType_t APP_SendError(Messages_ErrorTypeDef errorType) {
	Messages_TypeDef msg;

	msg.id = MSG_ID_ERROR;
	msg.length = sizeof(Messages_ErrorTypeDef);
	msg.data = &errorType; // On utilise un pointeur vers l'erreur, pas besoin d'allouer de mémoire ici;

	COM_USB_SendData(&msg);

	return pdTRUE;
}
