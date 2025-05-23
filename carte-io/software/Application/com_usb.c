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
#define HEADER_SIZE 3             // Taille de [SOF][Length][Type] => 3 octets

static void COM_USB_ReceiveCMDTask(void *pvParameters) ;

TaskHandle_t COM_USB_ReceiveCMDTaskhandle;

uint32_t COM_USB_UARTCircularBufferSize = APP_UART_CIRCULAR_BUFFER_SIZE;
uint8_t COM_USB_UARTCircularBuffer[APP_UART_CIRCULAR_BUFFER_SIZE];

static QueueHandle_t *ApplicationMessageQueue;

/**
 * @brief: Initialisation de la communication USB
 * @param AppMsgQueue: Pointeur vers la file de messages de l'application
 * @retval None
 */
void COM_USB_Init(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue!=NULL);

	ApplicationMessageQueue = AppMsgQueue;

	printf ("[I2C SensorsInit] Initialisation... ");

	/* Initialisation de l'uart COM_USB (USART_1) */
	UART_Config COM_USB_Config =
	{
			COM_USB_UARTCircularBuffer,
			COM_USB_UARTCircularBufferSize,
			10, 	// Période du timer de lecture, exprimé en ms (donc ici, 10 ms)
			3000000				// Pour l'instant, ne sert à rien, codé en dur par cubeMX
	};

	assert(UART_Init(UART_COM_USB, COM_USB_Config)==HAL_OK); // On verifie que l'init de l'uart s'est bien passée

	/* Création de la tâche FreeRTOS */
	xTaskCreate(COM_USB_ReceiveCMDTask,
			"COM_USB_ReceiveCmds",
			TASK_STACK_SIZE_STD,
			NULL,
			TASK_PRIO_COM_USB_RCV_CMD,
			&COM_USB_ReceiveCMDTaskhandle);
	vTaskResume(COM_USB_ReceiveCMDTaskhandle);

	printf ("Done\n");
}

/**
 * @brief  Envoie une trame de données sur le port USB
 * @param  msg: Pointeur vers le message à envoyer
 * @retval HAL_StatusTypeDef: Statut de l'envoi
 */
HAL_StatusTypeDef COM_USB_SendData(Messages_TypeDef *msg) {
	assert_param(msg != NULL);

	HAL_StatusTypeDef status = HAL_OK;
	uint16_t length = msg->length + HEADER_SIZE + 1; // +1 pour le checksum;
	uint8_t *frame_to_send = (uint8_t*)malloc(length * sizeof(uint8_t));

	if (frame_to_send == NULL) {
		printf("Erreur d'allocation mémoire pour l'envoi de données\n");
		return HAL_ERROR;
	}
	/* Remplir le buffer avec la trame à envoyer */
	frame_to_send[0] = SOF; // Start of Frame
	frame_to_send[1] = msg->length; // Longueur du champ 'data' de la trame, donc sans le header et sans le checksum
	frame_to_send[2] = msg->id; // Type de message

	memcpy(&frame_to_send[HEADER_SIZE], msg->data, msg->length); // Données du message

	uint8_t checksum = 0;
	for (int i = 0; i < msg->length - 1; i++) { // Calcul du checksum sur l'ensemble de la trame sauf le dernier octet
		checksum += frame_to_send[i];
	}

	frame_to_send[length-1] = -checksum; // Le checksum est le complément à 2 de la somme des octets

	// Envoi de la trame complète, timeout de 100ms, et suppression du buffer après l'envoi
	status=UART_Write(UART_COM_USB, frame_to_send, length, 100, UART_DeleteBuffer);

	return status;
}

/**
 * @brief  Tâche de réception de commandes
 * @param  pvParameters: Paramètres de la tâche (non utilisés ici)
 * @retval None
 */
void COM_USB_ReceiveCMDTask(void *pvParameters) {
	/* Buffers DMA et variables */
	uint8_t headerBuffer[HEADER_SIZE];

	for(;;) {
		// Attente de la réception du header d'une trame
		UART_Read(UART_COM_USB, headerBuffer, HEADER_SIZE, portMAX_DELAY); // attente infinie sur un header

		// Vérification du SOF
		if (headerBuffer[0] != SOF) {
			printf("[COM_USB_Receive] Invalid SOF/header\n");
			continue; // Trame invalide, on attend la suivante
		}

		// Lecture de la longueur du champ data de la trame
		uint8_t frameDataLength = headerBuffer[1];

		// Allocation dynamique sur la pile pour le reste de la trame
		uint8_t frameBuffer[frameDataLength+HEADER_SIZE+1]; // +1 pour le checksum, +HEADER_SIZE pour le header
		frameBuffer[0] = headerBuffer[0]; // SOF
		frameBuffer[1] = headerBuffer[1]; // Length
		frameBuffer[2] = headerBuffer[2]; // Type

		// Réception du reste de la trame
		UART_Read(UART_COM_USB, &frameBuffer[3], frameDataLength, 100); // attente de 100ms pour recevoir le reste de la trame

		// Calcul et vérification du checksum
		uint8_t calculatedChecksum = 0;
		for (int i = 0; i < frameDataLength + HEADER_SIZE + 1; i++) { // calcul sur l'ensemble de la trame, doit valoir zero
			calculatedChecksum += frameBuffer[i];
		}

		if (calculatedChecksum != 0) {
			printf("[COM_USB_Receive] Invalid checksum\n");
			continue;
		}

		// Traiter la trame reçue (par exemple : TYPE + DATA)
		Messages_TypeDef *message = NEW_MESSAGE(frameBuffer[2], NULL);
		if (frameDataLength > 0) {
			message->data = (uint8_t*) malloc(frameDataLength * sizeof(uint8_t));
			if (message->data == NULL) {
				printf("Erreur d'allocation mémoire pour le message\n");
				continue;
			}

			memcpy(message->data, &frameBuffer[3], frameDataLength);
		}

		/* Envoyer le message dans la file */
		if (message != NULL) {
			// la liberation de mémoire est gérée par la tâche qui reçoit le message
			if (xQueueSend(*ApplicationMessageQueue, (void*) &message, portMAX_DELAY) != pdPASS) {
				printf("Échec de l'envoi du message\n");
				DELETE_MESSAGE(message); // Libérer la mémoire en cas d'échec
			}
		}
	}
}

