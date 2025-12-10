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

/* Constantes */
#define SOF 0x7F                  // Start of Frame
#define HEADER_SIZE 2             // Taille de [SOF][Length][Type]

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(void*)

/* Handlers */
extern UART_HandleTypeDef huart1;

/* FreeRTOS */
//SemaphoreHandle_t APP_RXCompleteSemaphore;

/* Handle pour la file de messages */
QueueHandle_t APP_MessageQueue;

/* Prototypes */
void APP_ReceiveCMDTask(void *pvParameters) ;
void APP_MessageHandlerTask(void *pvParameters);
void processFrame(uint8_t type, uint8_t *data, uint8_t dataLength);
void APP_UART_RxCompleteCallback(UART_HandleTypeDef *huart);
void MX_USART1_UART_Init(void);

TaskHandle_t APP_ReceiveCMDTaskhandle;
TaskHandle_t APP_MessageHandlerTaskhandle;

/* Fonction principale */
void APP_Init(void) {

	MX_USART1_UART_Init();

	/* Initialisation USART et DMA */
	/* Activer l'utilisation des callbacks personnalisés */
	HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, APP_UART_RxCompleteCallback);

//	/* Création des sémaphores */
//	APP_RXCompleteSemaphore = xSemaphoreCreateBinary();
//	if (APP_RXCompleteSemaphore == NULL) {
//		printf("[APP Init] Erreur de création du semaphore\n");
//		while (1);
//	}
//	vQueueAddToRegistry(APP_RXCompleteSemaphore, "APP_RXCompleteSem" );

	APP_MessageQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	if (APP_MessageQueue == NULL) {
		printf("[APP Init] Erreur de création de la file\n");
		while (1);
	}
	vQueueAddToRegistry(APP_MessageQueue, "APP_MsgQ" );

	/* Création de la tâche FreeRTOS */
	xTaskCreate(APP_ReceiveCMDTask,
			"APP_ReceiveCmdS",
			TASK_STACK_SIZE_APPLICATION,
			NULL,
			TASK_PRIO_APP_RCV_CMD,
			&APP_ReceiveCMDTaskhandle);
	vTaskResume(APP_ReceiveCMDTaskhandle);

	/* Créer les tâches */
	xTaskCreate(APP_MessageHandlerTask,
			"APP_MsgHandler",
			TASK_STACK_SIZE_APPLICATION,
			NULL,
			TASK_PRIO_APP_MSG_HANDLER,
			&APP_MessageHandlerTaskhandle);
	vTaskResume(APP_MessageHandlerTaskhandle);
}

/* Fonction de la tâche qui traite les messages */
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

/* Callback personnalisé : completion totale du DMA */
void APP_UART_RxCompleteCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(APP_ReceiveCMDTaskhandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* Tâche FreeRTOS pour traiter les messages recus sur l'UART1 */
void APP_ReceiveCMDTask(void *pvParameters) {
	/* Buffers DMA et variables */
	uint8_t headerBuffer[HEADER_SIZE];
	char *message;

	while (1) {
		/* Démarrage du DMA */
		HAL_UART_Receive_DMA(&huart1, headerBuffer, HEADER_SIZE);

		// Attente que la réception DMA soit terminée
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Vérification du SOF
		if (headerBuffer[0] != SOF) {
			printf("[APP_Receive] Invalid SOF/header\n");
			continue; // Trame invalide, on attend la suivante
		}

		// Lecture de la longueur de la trame
		uint8_t frameLength = headerBuffer[1];

		// Allocation dynamique sur la pile pour le reste de la trame
		uint8_t frameBuffer[frameLength];

		// Réception du reste de la trame
		HAL_UART_Receive_DMA(&huart1, frameBuffer, frameLength);

		// Attente que la réception DMA soit terminée
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Calcul et vérification du checksum
		uint8_t calculatedChecksum = 0;
		for (int i = 0; i < frameLength; i++) {
			calculatedChecksum += frameBuffer[i];
		}
		if (calculatedChecksum != 0) {
			printf("[APP_Receive] Invalid checksum\n");
			continue;
		}

		// Traiter la trame reçue (par exemple : TYPE + DATA)
		uint8_t type = frameBuffer[0];
		uint8_t *data = &frameBuffer[1];

		// Traitement de la trame selon le type
		processFrame(type, data, frameLength - 2);

		// envoi du message à la tache messagehandler (test)
		/* Allouer dynamiquement de la mémoire pour un message */
		message = (char *)malloc(50 * sizeof(char));
		if (message == NULL) {
			printf("Erreur d'allocation mémoire\n");
			continue;
		}

		/* Remplir le message avec des données */
		snprintf(message, 50, "Msg:\n\tType=%d\n\tLength=%d\n", type, frameLength-2);

		/* Envoyer le message dans la file */
		if (xQueueSend(APP_MessageQueue, &message, portMAX_DELAY) != pdPASS) {
			printf("Échec de l'envoi du message\n");
			free(message); // Libérer la mémoire en cas d'échec
		}
	}
}

/* Traitement d'un message valide */
void processFrame(uint8_t type, uint8_t *data, uint8_t dataLength) {
	// Traitement spécifique au type
	printf("Message Type: %d, Data Length: %d\n", type, dataLength);
	printf("Message data: %d\n", *data);
}






