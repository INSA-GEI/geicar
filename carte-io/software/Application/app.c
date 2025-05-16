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

/* Constantes */
#define SOF 0x7F                  // Start of Frame
#define HEADER_SIZE 2             // Taille de [SOF][Length][Type]

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(void*)

/* Handlers */
extern UART_HandleTypeDef huart1;
UART_Handle APP_UartHandle;

/* Handle pour la file de messages */
QueueHandle_t APP_MessageQueue;

/* Prototypes */
void APP_ReceiveCMDTask(void *pvParameters) ;
void APP_MessageHandlerTask(void *pvParameters);
void processFrame(uint8_t type, uint8_t *data, uint8_t dataLength);
void APP_UART_RxCallback(UART_HandleTypeDef *huart);

TaskHandle_t APP_ReceiveCMDTaskhandle;
TaskHandle_t APP_MessageHandlerTaskhandle;

uint32_t APP_UARTCircularBufferSize = APP_UART_CIRCULAR_BUFFER_SIZE;
uint8_t APP_UARTCircularBuffer[APP_UART_CIRCULAR_BUFFER_SIZE];

/* Fonction principale */
void APP_Init(void) {

	printf ("[APP_Init] Initialisation... ");

	/* Initialisation de l'uart 1 */
	UART_Config appUARTConfig =
	{
			APP_UARTCircularBuffer,
			APP_UARTCircularBufferSize,
			10, 	// Période du timer de lecture, exprimé en ms (donc ici, 10 ms)
			3000000				// Pour l'instant, ne sert à rien, codé en dur par cubeMX
	};

	assert(UART_Init(&APP_UartHandle, USART1,  appUARTConfig)==HAL_OK); // On verifie que l'init de l'uart s'est bien passée

	APP_MessageQueue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	if (APP_MessageQueue == NULL) {
		printf("[APP Init] Erreur de création de la file\n");
		while (1);
	}
	vQueueAddToRegistry(APP_MessageQueue, "APP_MsgQ" );

	/* Création de la tâche FreeRTOS */
	xTaskCreate(APP_ReceiveCMDTask,
			"APP_ReceiveCmds",
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

	printf ("Done\n");
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

/* Tâche FreeRTOS pour traiter les messages recus sur l'UART1 */
void APP_ReceiveCMDTask(void *pvParameters) {
	/* Buffers DMA et variables */
	uint8_t headerBuffer[HEADER_SIZE];
	char *message;

	while (1) {

		// debug du driver d'uart
		uint8_t bufferTest[26]={0}; // 25 Caractères + 0 terminal

		//message = (char *)malloc(50 * sizeof(char)); // allocation avec que des zeros
		while (1) {
			//memset(message, 0, 50);

			if (UART_Read(&APP_UartHandle, bufferTest, 25, portMAX_DELAY) == HAL_OK) {
				//snprintf(message, 50, "[APP_Receive] Msg reçu: %s\n", bufferTest);
				//printf(message);
				UART_Write(&APP_UartHandle, bufferTest, 25, 100, 0);
			} else {
				printf("[APP_Receive] Échec du test de reception uart\n");
			}
		}

		UART_Read(&APP_UartHandle, headerBuffer, HEADER_SIZE, portMAX_DELAY); // attente infinie sur un header

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
		UART_Read(&APP_UartHandle, frameBuffer, frameLength, 100); // attente de 100ms pour recevoir le reste de la trame
		//HAL_UART_Receive_DMA(&huart1, frameBuffer, frameLength);

		// Attente que la réception DMA soit terminée
		//ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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






