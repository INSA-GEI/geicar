/*
 * com-usb.c
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "messages.h"

#include "Services/uartdrv.h"

/* Constantes */
#define SOF 0x7F                  // Start of Frame
#define HEADER_SIZE 2             // Taille de [SOF][Length][Type]

static void COM_USB_ReceiveCMDTask(void *pvParameters) ;
static void processFrame(uint8_t type, uint8_t *data, uint8_t dataLength);

TaskHandle_t COM_USB_ReceiveCMDTaskhandle;

uint32_t COM_USB_UARTCircularBufferSize = APP_UART_CIRCULAR_BUFFER_SIZE;
uint8_t COM_USB_UARTCircularBuffer[APP_UART_CIRCULAR_BUFFER_SIZE];

static QueueHandle_t *ApplicationMessageQueue;

void COM_USB_Init(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue!=NULL);

	ApplicationMessageQueue = AppMsgQueue;

	/* Initialisation de l'uart 1 */
	UART_Config COM_USB_Config =
	{
			COM_USB_UARTCircularBuffer,
			COM_USB_UARTCircularBufferSize,
			10, 	// Période du timer de lecture, exprimé en ms (donc ici, 10 ms)
			3000000				// Pour l'instant, ne sert à rien, codé en dur par cubeMX
	};

	assert(UART_Init(USART1,  COM_USB_Config)==HAL_OK); // On verifie que l'init de l'uart s'est bien passée

	/* Création de la tâche FreeRTOS */
	xTaskCreate(COM_USB_ReceiveCMDTask,
			"COM_USB_ReceiveCmds",
			TASK_STACK_SIZE_APPLICATION,
			NULL,
			TASK_PRIO_COM_USB_RCV_CMD,
			&COM_USB_ReceiveCMDTaskhandle);
	vTaskResume(COM_USB_ReceiveCMDTaskhandle);
}

/**
 * @brief  Tâche de réception de commandes
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void COM_USB_ReceiveCMDTask(void *pvParameters) {
	/* Buffers DMA et variables */
	uint8_t headerBuffer[HEADER_SIZE];

	while (1) {
		// Attente de la réception du header d'une trame
		UART_Read(UART_COM_USB, headerBuffer, HEADER_SIZE, portMAX_DELAY); // attente infinie sur un header

		// Vérification du SOF
		if (headerBuffer[0] != SOF) {
			printf("[COM_USB_Receive] Invalid SOF/header\n");
			continue; // Trame invalide, on attend la suivante
		}

		// Lecture de la longueur de la trame
		uint8_t frameLength = headerBuffer[1];

		// Allocation dynamique sur la pile pour le reste de la trame
		uint8_t frameBuffer[frameLength];

		// Réception du reste de la trame
		UART_Read(UART_COM_USB, frameBuffer, frameLength, 100); // attente de 100ms pour recevoir le reste de la trame

		// Calcul et vérification du checksum
		uint8_t calculatedChecksum = 0;
		for (int i = 0; i < frameLength; i++) {
			calculatedChecksum += frameBuffer[i];
		}
		if (calculatedChecksum != 0) {
			printf("[COM_USB_Receive] Invalid checksum\n");
			continue;
		}

		// Traiter la trame reçue (par exemple : TYPE + DATA)
		uint8_t type = frameBuffer[0];
		uint8_t *data = &frameBuffer[1];

		// Traitement de la trame selon le type
		processFrame(type, data, frameLength - 2);

		// envoi du message à la tache messagehandler (test)
		/* Allouer dynamiquement de la mémoire pour un message */

		Messages_TypeDef *message = NEW_MESSAGE(MSG_ID_STRING, NULL);
		message->length = 50;
		message->data = (uint8_t *)malloc(message->length * sizeof(char));

		if (message->data == NULL) {
			printf("Erreur d'allocation mémoire\n");
			continue;
		}

		/* Remplir le message avec des données */
		snprintf((char*)message->data, message->length, "Msg:\n\tType=%d\n\tLength=%d\n", type, frameLength-2);

		/* Envoyer le message dans la file */
		if (xQueueSend(*ApplicationMessageQueue, message, portMAX_DELAY) != pdPASS) {
			printf("Échec de l'envoi du message\n");
			DELETE_MESSAGE(message); // Libérer la mémoire en cas d'échec
		}
	}
}

/**
 * @brief  Traite un message reçu
 * @param  type: Type de message
 * @param  data: Données du message
 * @param  dataLength: Longueur des données
 * @retval None
 */
void processFrame(uint8_t type, uint8_t *data, uint8_t dataLength) {
	// Traitement spécifique au type
	printf("Message Type: %d, Data Length: %d\n", type, dataLength);
	printf("Message data: %d\n", *data);
}
