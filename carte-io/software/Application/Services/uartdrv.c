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

// Externes déclarés dans le projet
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

/**
 * @brief  Contexte du driver d'UART
 */
typedef struct {
	UART_HandleTypeDef *huart;    // UART handle (déjà initialisé par MX_USARTx_UART_Init)
    UART_Config config;
    SemaphoreHandle_t tx_semaphore;
    SemaphoreHandle_t rx_semaphore;

    UART_DeleteEnum deleteAfterSend;
    uint8_t* txBuffer;

    TimerHandle_t rxPeriodicTimer;          // Timer pour surveiller la réception

    uint8_t readInProgress;
    uint8_t* rxBuffer;
    uint32_t rxBufferSize;
    uint32_t rxBufferWriteIndex;

    uint32_t dmaReadIndex;
    uint8_t counter;
} UART_Context;

typedef enum {
	USART_DEV_1 = 0,
	USART_DEV_3,
	UART_DEV_4,
	UART_DEV_5,
	LPUART_DEV_1,
	UART_DEV_COUNT
} UART_Device;

static UART_Context uart_contexts[UART_DEV_COUNT] = {0};

static bool UART_proceedCircularDMA(UART_Context *ctx, uint32_t currentDMAIndex);
static void onTXEvent(UART_HandleTypeDef *huart);
static void onRXEvent(UART_HandleTypeDef *huart);
static void onErrorEvent(UART_HandleTypeDef *huart);
static void onTimerEvent(TimerHandle_t xTimer);

//static UART_Context* USART1_handle;
//static UART_Context* USART3_handle;
//static UART_Context* UART4_handle;
//static UART_Context* UART5_handle;
//static UART_Context* LPUART1_handle;

/**
 * @brief  Recupere un context UART en fonction de l'instance
 * @param  instance: Instance de l'UART
 * @retval Contexte de l'UART
 */
static UART_Context *UART_GetContext(USART_TypeDef *instance) {
	UART_Context *ctx =NULL;

	switch ((uint32_t) instance) {
	case (uint32_t) USART1:
		ctx= &uart_contexts[USART_DEV_1];
		break;
	case (uint32_t) USART3:
		ctx= &uart_contexts[USART_DEV_3];
		break;
	case (uint32_t) UART4:
		ctx= &uart_contexts[UART_DEV_4];
		break;
	case (uint32_t) UART5:
		ctx= &uart_contexts[UART_DEV_5];
		break;
	case (uint32_t) LPUART1:
		ctx= &uart_contexts[LPUART_DEV_1];
		break;
	default:
		assert_param(0); // TODO: Implement error handling
		break;
	}

	return ctx;
}

/**
 * @brief  Initialisation de l'UART, configuration en mode DMA circulaire pour la reception et DMA standard pour l'envoi
 * @param  uart: Instance d'UART à configurer
 * @param  config: Configuration de l'UART
 * @retval HAL_OK si l'initialisation s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Init(USART_TypeDef *instance, UART_Config config) {
	if (!instance || !instance || !config.rxCircularBuffer || config.rxCircularBufferSize == 0)
		return HAL_ERROR;

	char *sem_tx_name;
	char *sem_rx_name;
    char *timer_name;

	UART_Context *ctx = UART_GetContext(instance);

	ctx->config = config;
	ctx->rxBufferWriteIndex = 0;
	ctx->rxBufferSize = 0;

	/* Reset l'UART si il a déjà été configuré */
	if ((ctx->huart != 0) && (ctx->huart->Instance != 0))
		HAL_UART_DeInit(ctx->huart);

	/* Configuration de l'UART */
	/* TODO: Réfléchir a un moyen de changer le baudrate */
	switch ((uint32_t)instance) {
	case (uint32_t)USART1:
		MX_USART1_UART_Init();
		ctx->huart=&huart1;
		sem_tx_name = "SEM USART1 TX";
		sem_rx_name = "SEM USART1 RX";
		timer_name = "TIMER USART1";
		break;
	case (uint32_t)USART3:
		MX_USART3_UART_Init();
		ctx->huart=&huart3;
		sem_tx_name = "SEM USART3 TX";
		sem_rx_name = "SEM USART3 RX";
		timer_name = "TIMER USART3";
		break;
	case (uint32_t)UART4:
		MX_UART4_Init();
		ctx->huart=&huart4;
		sem_tx_name = "SEM UART4 TX";
		sem_rx_name = "SEM UART4 RX";
		timer_name = "TIMER UART4";
		break;
	case (uint32_t)UART5:
		MX_UART5_Init();
		ctx->huart=&huart5;
		sem_tx_name = "SEM UART5 TX";
		sem_rx_name = "SEM UART5 RX";
		timer_name = "TIMER UART5";
		break;
	case (uint32_t)LPUART1:
		MX_LPUART1_UART_Init();
		ctx->huart= &hlpuart1;
		sem_tx_name = "SEM LPUART1 TX";
		sem_rx_name = "SEM LPUART1 RX";
		timer_name = "TIMER LPUART1";
		break;
	default:
		return HAL_ERROR;
	}

	// Creation des semaphores
	ctx->tx_semaphore = xSemaphoreCreateBinary();
	assert_param(ctx->tx_semaphore != NULL);

	ctx->rx_semaphore = xSemaphoreCreateBinary();
	assert_param(ctx->rx_semaphore != NULL);

	/*
	 * le semaphore TX doit être à 1 pour pouvoir être pris
	 * par la méthode write en entrant: cela évite la reantrance dans la fonction write
	 * tant qu'elle n'a pas finie
	 *
	 * A l'inverse, le semaphore RX doit être à l'etat 0 en entrant dans la méthode
	 * read pour bloquer tant que les données n'ont pas été reçues (et donc que le
	 * semaphore RX soit produit)
	 */
	xSemaphoreGive(ctx->tx_semaphore);
	vQueueAddToRegistry(ctx->tx_semaphore, sem_tx_name);
	vQueueAddToRegistry(ctx->rx_semaphore, sem_rx_name);

	ctx->rxPeriodicTimer = xTimerCreate(timer_name,           // Nom du timer
			pdMS_TO_TICKS(ctx->config.rxPeriodicTimerDelay),    // Période en ticks ( ex 500 ms)
			pdTRUE,       // Auto-reload (pdTRUE = répète, pdFALSE = unique)
			(void*) ctx,             // contexte de l'instance, pour récupérer la conf lors de l'appel
			onTimerEvent      // Fonction callback
	);
	assert_param(ctx->rxPeriodicTimer != NULL);

	/* Démarre la reception en DMA circulaire */
	HAL_StatusTypeDef status = HAL_UART_Receive_DMA(ctx->huart, config.rxCircularBuffer, config.rxCircularBufferSize);
	if (status == HAL_OK) {
		HAL_UART_RegisterCallback(ctx->huart,
				HAL_UART_TX_COMPLETE_CB_ID, onTXEvent);
		HAL_UART_RegisterCallback(ctx->huart,
				HAL_UART_RX_HALFCOMPLETE_CB_ID, onRXEvent);
		HAL_UART_RegisterCallback(ctx->huart,
				HAL_UART_RX_COMPLETE_CB_ID, onRXEvent);
		HAL_UART_RegisterCallback(ctx->huart,
				HAL_UART_ERROR_CB_ID, onErrorEvent);
	}

	return status;
}

/**
 * @brief  Envoi de données sur l'UART. Le buffer peut être supprimé ou non apres l'envoi (deleteAfterSend)
 *         et un timeout peut-être indiqué (ou mis à portMAX_DELAY si l'on veut une attente infinie)
 * @param  instance: instance de l'UART
 * @param  data: Données à envoyer
 * @param  size: Taille des données à envoyer
 * @param  timeout: Timeout d'attente du semaphore TX (portMAX_DELAY pour une attente infinie)
 * @param  deleteAfterSend: Indique si le buffer doit être libéré après l'envoi (UART_DeleteBuffer) ou non (UART_KeepBuffer)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Write(USART_TypeDef *instance, uint8_t *data, uint16_t size, uint32_t timeout, UART_DeleteEnum deleteAfterSend) {
	if (!instance || !data || size == 0) {
		return HAL_ERROR;
	}

	UART_Context *ctx = UART_GetContext(instance);

	HAL_StatusTypeDef status = HAL_ERROR;
	TickType_t timeoutFreertos = portMAX_DELAY;

	if (timeout!=portMAX_DELAY)
		timeoutFreertos = pdMS_TO_TICKS(timeout);

	/* On attend que l'UART soit libre */
	if (xSemaphoreTake(ctx->tx_semaphore, timeoutFreertos) == pdTRUE) {
		// Enregistrement de la demande de liberation mémoire en fin de transfert
		ctx->deleteAfterSend = deleteAfterSend;
		ctx->txBuffer = data;

		status = HAL_UART_Transmit_DMA(ctx->huart, data, size);
	} else
		status = HAL_TIMEOUT;

	return status;
}

/**
 * @brief  Lecture de données sur l'UART. Le buffer doit être alloué par l'utilisateur et la taille indiquée
 *         ainsi qu'un timeout (portMAX_DELAY pour une attente infinie)
 * @param  instance: Instance de l'UART
 * @param  buffer: Buffer de réception
 * @param  length: Taille du buffer
 * @param  timeout: Timeout d'attente du semaphore RX (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si la lecture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Read(USART_TypeDef *instance, uint8_t *buffer, uint16_t length, uint32_t timeout) {
	if (!instance || !buffer || length == 0)
		return HAL_ERROR;

	UART_Context *ctx = UART_GetContext(instance);

	HAL_StatusTypeDef status;
	BaseType_t semStatus = pdFALSE;

	ctx->rxBuffer = buffer;
	ctx->rxBufferSize = length;
	ctx->rxBufferWriteIndex = 0;
	ctx->readInProgress = 1;
	ctx->counter=0;

	// Démarrage du timer / lecture périodique
	assert_param(xTimerStart(ctx->rxPeriodicTimer,0) == pdPASS);

	// on part du principe que tout va bien se passer
	status = HAL_OK;

	// attente du semaphore de fin de lecture
	if (timeout != portMAX_DELAY) {
		if (xSemaphoreTake(ctx->rx_semaphore, pdMS_TO_TICKS(timeout))!=pdTRUE)
			status = HAL_TIMEOUT;
	} else {
		// Attente infinie tant que le semaphore n'est pas produit
		while (semStatus != pdTRUE) {
			semStatus = xSemaphoreTake(ctx->rx_semaphore, portMAX_DELAY);
		}
	}

	return status;
}

/**
 * @brief  Gestion du buffer circulaire de reception. Appelé soit par les evenements DMA (half ou complete), soit par le timer periodique
 * @param  ctx: Contexte de l'UART
 * @param  currentDMAIndex: Index actuel de la DMA
 * @retval true si les données attendues ont toutes été reçues et copié dans le buffer de lecture, false sinon
 */
static bool UART_proceedCircularDMA(UART_Context *ctx, uint32_t currentDMAIndex) {
	if (ctx->readInProgress) {
		if (ctx->rxBufferWriteIndex >= ctx->rxBufferSize)
			return 1; // Si déjà rempli, ne rien faire

		size_t dmaWriteIndex = (size_t) ctx->config.rxCircularBufferSize - (size_t) currentDMAIndex;
		// L'index DMA va de la taille du buffer à 0 (décrément)
		// ainsi, si le buffer a une taille de 50 et l'index vaut 48
		// il n'y a que 50-48 =2 octets dans le buffer

		size_t availableData =
				(dmaWriteIndex >= ctx->dmaReadIndex) ?
						(dmaWriteIndex - ctx->dmaReadIndex) :
						(ctx->config.rxCircularBufferSize - ctx->dmaReadIndex + dmaWriteIndex);

		while (availableData > 0 && ctx->rxBufferWriteIndex < ctx->rxBufferSize) {
			ctx->rxBuffer[ctx->rxBufferWriteIndex] = ctx->config.rxCircularBuffer[ctx->dmaReadIndex];
			ctx->dmaReadIndex = (ctx->dmaReadIndex + 1) % ctx->config.rxCircularBufferSize;
			ctx->rxBufferWriteIndex++;
			availableData--;

			if (ctx->rxBufferWriteIndex == ctx->rxBufferSize) {
				// On a reçu nos données, arrêt du timer périodique et on indique que l'on n'est plus en phase de reception
				xTimerStop(ctx->rxPeriodicTimer, 0);
				ctx->readInProgress = 0;
				return true;
			}
		}
	}

	return false;
}

/**
 * @brief  Fonction de callback du timer périodique. Appelée à chaque fois que le timer expire
 * @param  xTimer: Handle du timer
 * @retval None
 */
static void onTimerEvent(TimerHandle_t xTimer) {
	UART_Context *ctx = (UART_Context*)pvTimerGetTimerID(xTimer);

	if (ctx) {
		/*if (UART_GetRxAvailable(handle) >= handle->rx_expected_length) {
			xSemaphoreGive(handle->rx_semaphore);*/
		onRXEvent(ctx->huart);
	}
}

/**
 * @brief  Fonction de callback de fin de transmission de l'UART. Appelée à chaque fois que le DMA a fini l'envoi d'un buffer
 * @param  huart: Handle de l'UART
 * @retval None
 */
static void onTXEvent(UART_HandleTypeDef *huart) {
	UART_Context *ctx = UART_GetContext(huart->Instance);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// liberation mémoire du buffer TX si demandé lors de l'envoi
	if ((ctx->deleteAfterSend==UART_DeleteBuffer) && (ctx->txBuffer))
		free(ctx->txBuffer);

	/* Liberation du semaphore TX */
	xSemaphoreGiveFromISR(ctx->tx_semaphore, &xHigherPriorityTaskWoken);

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
	UART_Context *ctx = UART_GetContext(huart->Instance);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t status;

	status = UART_proceedCircularDMA(ctx, __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx));
	if (status) {
		/* Liberation du semaphore RX */
		xSemaphoreGiveFromISR(ctx->rx_semaphore, &xHigherPriorityTaskWoken);

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
