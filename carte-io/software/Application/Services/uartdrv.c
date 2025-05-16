/*
 * uartdrv.c
 *
 *  Created on: May 5, 2025
 *      Author: dimercur
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "uartdrv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

extern void MX_LPUART1_UART_Init(void);
extern void MX_UART4_Init(void);
extern void MX_UART5_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART3_UART_Init(void);

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

static bool UART_proceedCircularDMA(UART_Handle *handle, uint32_t currentDMAIndex);
static UART_Handle* UART_GetHandleFromUART(UART_HandleTypeDef *huart);
static void onTXEvent(UART_HandleTypeDef *huart);
static void onRXEvent(UART_HandleTypeDef *huart);
static void onErrorEvent(UART_HandleTypeDef *huart);
static void onTimerEvent(TimerHandle_t xTimer);

static UART_Handle* USART1_handle;
static UART_Handle* USART3_handle;
static UART_Handle* UART4_handle;
static UART_Handle* UART5_handle;
static UART_Handle* LPUART1_handle;

/**
 * @brief  Initialisation de l'UART, configuration en mode DMA circulaire pour la reception et DMA standard pour l'envoi
 * @param  handle: Handle de l'UART
 * @param  uart: Instance d'UART à configurer
 * @param  config: Configuration de l'UART
 * @retval HAL_OK si l'initialisation s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Init(UART_Handle *handle, USART_TypeDef *uart, UART_Config config) {
	if (!handle || !uart || !config.rxCircularBuffer || config.rxCircularBufferSize == 0)
		return HAL_ERROR;

	handle->config = config;
	handle->rxBufferWriteIndex = 0;
	handle->rxBufferSize = 0;

	/* Reset l'UART si il a déjà été configuré */
	if ((handle->huart != 0) && (handle->huart->Instance != 0))
		HAL_UART_DeInit(handle->huart);

	/* Configuration de l'UART */
	/* TODO: Réfléchir a un moyen de changer le baudrate */
	switch ((uint32_t)uart) {
	case (uint32_t)USART1:
			MX_USART1_UART_Init();
	handle->huart=&huart1;
	USART1_handle = handle;
	break;
	case (uint32_t)USART3:
			MX_USART3_UART_Init();
	handle->huart=&huart3;
	USART3_handle = handle;
	break;
	case (uint32_t)UART4:
			MX_UART4_Init();
	handle->huart=&huart4;
	UART4_handle = handle;
	break;
	case (uint32_t)UART5:
			MX_UART5_Init();
	handle->huart=&huart5;
	UART5_handle = handle;
	break;
	case (uint32_t)LPUART1:
			MX_LPUART1_UART_Init();
	handle->huart= &hlpuart1;
	LPUART1_handle = handle;
	break;
	default:
		return HAL_ERROR;
	}

	// Creation des semaphores
	handle->tx_semaphore = xSemaphoreCreateBinary();
	assert_param(handle->tx_semaphore != NULL);

	handle->rx_semaphore = xSemaphoreCreateBinary();
	assert_param(handle->rx_semaphore != NULL);

	/*
	 * le semaphore TX doit être à 1 pour pouvoir être pris
	 * par la méthode write en entrant: cela évite la reantrance dans la fonction write
	 * tant qu'elle n'a pas finie
	 *
	 * A l'inverse, le semaphore RX doit être à l'etat 0 en entrant dans la méthode
	 * read pour bloquer tant que les données n'ont pas été reçues (et donc que le
	 * semaphore RX soit produit)
	 */
	xSemaphoreGive(handle->tx_semaphore);
	vQueueAddToRegistry(handle->tx_semaphore, "TX Complete");
	vQueueAddToRegistry(handle->rx_semaphore, "RX Complete");

	handle->rxPeriodicTimer = xTimerCreate("UART_Timer",           // Nom du timer
			pdMS_TO_TICKS(handle->config.rxPeriodicTimerDelay),    // Période en ticks ( ex 500 ms)
			pdTRUE,       // Auto-reload (pdTRUE = répète, pdFALSE = unique)
			(void*) handle,             // handle de l'uart, pour récupérer la conf lors de l'appel
			onTimerEvent      // Fonction callback
	);
	assert_param(handle->rxPeriodicTimer != NULL);

	/* Démarre la reception en DMA circulaire */
	HAL_StatusTypeDef status = HAL_UART_Receive_DMA(handle->huart, config.rxCircularBuffer, config.rxCircularBufferSize);
	if (status == HAL_OK) {
		HAL_UART_RegisterCallback(handle->huart,
				HAL_UART_TX_COMPLETE_CB_ID, onTXEvent);
		HAL_UART_RegisterCallback(handle->huart,
				HAL_UART_RX_HALFCOMPLETE_CB_ID, onRXEvent);
		HAL_UART_RegisterCallback(handle->huart,
				HAL_UART_RX_COMPLETE_CB_ID, onRXEvent);
		HAL_UART_RegisterCallback(handle->huart,
				HAL_UART_ERROR_CB_ID, onErrorEvent);
	}

	return status;
}

/**
 * @brief  Envoi de données sur l'UART. Le buffer peut être supprimé ou non apres l'envoi (deleteAfterSend)
 *         et un timeout peut-être indiqué (ou mis à portMAX_DELAY si l'on veut une attente infinie)
 * @param  handle: Handle de l'UART
 * @param  data: Données à envoyer
 * @param  size: Taille des données à envoyer
 * @param  timeout: Timeout d'attente du semaphore TX (portMAX_DELAY pour une attente infinie)
 * @param  deleteAfterSend: Indique si le buffer doit être libéré après l'envoi (UART_DeleteBuffer) ou non (UART_KeepBuffer)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Write(UART_Handle *handle, uint8_t *data, uint16_t size, uint32_t timeout, UART_DeleteEnum deleteAfterSend) {
	if (!handle || !data || size == 0) {
		return HAL_ERROR;
	}

	HAL_StatusTypeDef status = HAL_ERROR;
	TickType_t timeoutFreertos = portMAX_DELAY;

	if (timeout!=portMAX_DELAY)
		timeoutFreertos = pdMS_TO_TICKS(timeout);

	/* On attend que l'UART soit libre */
	if (xSemaphoreTake(handle->tx_semaphore, timeoutFreertos) == pdTRUE) {
		// Enregistrement de la demande de liberation mémoire en fin de transfert
		handle->deleteAfterSend = deleteAfterSend;
		handle->txBuffer = data;

		status = HAL_UART_Transmit_DMA(handle->huart, data, size);
	} else
		status = HAL_TIMEOUT;

	return status;
}

/**
 * @brief  Lecture de données sur l'UART. Le buffer doit être alloué par l'utilisateur et la taille indiquée
 *         ainsi qu'un timeout (portMAX_DELAY pour une attente infinie)
 * @param  handle: Handle de l'UART
 * @param  buffer: Buffer de réception
 * @param  length: Taille du buffer
 * @param  timeout: Timeout d'attente du semaphore RX (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si la lecture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Read(UART_Handle *handle, uint8_t *buffer, uint16_t length, uint32_t timeout) {
	if (!handle || !buffer || length == 0)
		return HAL_ERROR;

	HAL_StatusTypeDef status;
	BaseType_t semStatus = pdFALSE;

	handle->rxBuffer = buffer;
	handle->rxBufferSize = length;
	handle->rxBufferWriteIndex = 0;
	handle->readInProgress = 1;
	handle->counter=0;

	// Démarrage du timer / lecture périodique
	assert_param(xTimerStart(handle->rxPeriodicTimer,0) == pdPASS);

	// on part du principe que tout va bien se passer
	status = HAL_OK;

	// attente du semaphore de fin de lecture
	if (timeout != portMAX_DELAY) {
		if (xSemaphoreTake(handle->rx_semaphore, pdMS_TO_TICKS(timeout))!=pdTRUE)
			status = HAL_TIMEOUT;
	} else {
		// Attente infinie tant que le semaphore n'est pas produit
		while (semStatus != pdTRUE) {
			semStatus = xSemaphoreTake(handle->rx_semaphore, portMAX_DELAY);
		}
	}

	return status;
}

/**
 * @brief  Gestion du buffer circulaire de reception. Appelé soit par les evenements DMA (half ou complete), soit par le timer periodique
 * @param  handle: Handle de l'UART
 * @param  currentDMAIndex: Index actuel de la DMA
 * @retval true si les données attendues ont toutes été reçues et copié dans le buffer de lecture, false sinon
 */
static bool UART_proceedCircularDMA(UART_Handle *handle, uint32_t currentDMAIndex) {
	if (handle->readInProgress) {
		if (handle->rxBufferWriteIndex >= handle->rxBufferSize)
			return 1; // Si déjà rempli, ne rien faire

		size_t dmaWriteIndex = (size_t) handle->config.rxCircularBufferSize - (size_t) currentDMAIndex;
		// L'index DMA va de la taille du buffer à 0 (décrément)
		// ainsi, si le buffer a une taille de 50 et l'index vaut 48
		// il n'y a que 50-48 =2 octets dans le buffer

		size_t availableData =
				(dmaWriteIndex >= handle->dmaReadIndex) ?
						(dmaWriteIndex - handle->dmaReadIndex) :
						(handle->config.rxCircularBufferSize - handle->dmaReadIndex + dmaWriteIndex);

		while (availableData > 0 && handle->rxBufferWriteIndex < handle->rxBufferSize) {
			handle->rxBuffer[handle->rxBufferWriteIndex] = handle->config.rxCircularBuffer[handle->dmaReadIndex];
			handle->dmaReadIndex = (handle->dmaReadIndex + 1) % handle->config.rxCircularBufferSize;
			handle->rxBufferWriteIndex++;
			availableData--;

			if (handle->rxBufferWriteIndex == handle->rxBufferSize) {
				// On a reçu nos données, arrêt du timer périodique et on indique que l'on n'est plus en phase de reception
				xTimerStop(handle->rxPeriodicTimer, 0);
				handle->readInProgress = 0;
				return true;
			}
		}
	}

	return false;
}

/**
 * @brief  Converti un handle d'UART (renvoyé par les focntions callback) en handle du driver UART
 * @param  huart: Handle de l'UART
 * @retval handle du driver UART
 */
static UART_Handle* UART_GetHandleFromUART(UART_HandleTypeDef *huart) {
	UART_Handle *handle= NULL;

	switch ((uint32_t)huart->Instance) {
	case (uint32_t)USART1:
			handle = USART1_handle;
	break;
	case (uint32_t)USART3:
			handle = USART3_handle;
	break;
	case (uint32_t)UART4:
			handle = UART4_handle;
	break;
	case (uint32_t)UART5:
			handle = UART5_handle;
	break;
	case (uint32_t)LPUART1:
			handle = LPUART1_handle;
	break;
	default:
		assert_param(0);
	}

	return handle;
}

/**
 * @brief  Fonction de callback du timer périodique. Appelée à chaque fois que le timer expire
 * @param  xTimer: Handle du timer
 * @retval None
 */
static void onTimerEvent(TimerHandle_t xTimer) {
	UART_Handle *handle = (UART_Handle*)pvTimerGetTimerID(xTimer);

	if (handle) {
		/*if (UART_GetRxAvailable(handle) >= handle->rx_expected_length) {
			xSemaphoreGive(handle->rx_semaphore);*/
		onRXEvent(handle->huart);
	}
}

/**
 * @brief  Fonction de callback de fin de transmission de l'UART. Appelée à chaque fois que le DMA a fini l'envoi d'un buffer
 * @param  huart: Handle de l'UART
 * @retval None
 */
static void onTXEvent(UART_HandleTypeDef *huart) {
	UART_Handle *handle = UART_GetHandleFromUART(huart);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// liberation mémoire du buffer TX si demandé lors de l'envoi
	if ((handle->deleteAfterSend==UART_DeleteBuffer) && (handle->txBuffer))
		free(handle->txBuffer);

	/* Liberation du semaphore TX */
	xSemaphoreGiveFromISR(handle->tx_semaphore, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The
		 actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  Fonction de callback de fin de reception de l'UART. Appelée à chaque fois que le DMA a atteint le milieu ou la fin du buffer circulaire
 * @param  huart: Handle de l'UART
 * @retval None
 */
static void onRXEvent(UART_HandleTypeDef *huart) {
	UART_Handle *handle = UART_GetHandleFromUART(huart);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t status;

	status = UART_proceedCircularDMA(handle, __HAL_DMA_GET_COUNTER(handle->huart->hdmarx));
	if (status) {
		/* Liberation du semaphore RX */
		xSemaphoreGiveFromISR(handle->rx_semaphore, &xHigherPriorityTaskWoken);

		/* Yield if xHigherPriorityTaskWoken is true. The actual macro used here is port specific. */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * @brief  Fonction de callback d'erreur de l'UART. Appelée à chaque fois qu'une erreur se produit sur l'UART
 * @param  huart: Handle de l'UART
 * @retval None
 */
static void onErrorEvent(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle //
	//UART_Handle *handle = UART_GetHandleFromUART(huart);
	assert_param(0);
}
