/*
 * i2cdrv.c
 *
 *  Created on: May 16, 2025
 *      Author: dimercur
 */

#include "i2cdrv.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Externes déclarés dans le projet
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

extern void MX_I2C1_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_I2C4_Init(void);

static void onTXEvent(I2C_HandleTypeDef *hi2c);
static void onRXEvent(I2C_HandleTypeDef *hi2c);
static void onErrorEvent(I2C_HandleTypeDef *hi2c);
static void onAbortEvent(I2C_HandleTypeDef *hi2c);

typedef struct {
	I2C_HandleTypeDef *hi2c;
	SemaphoreHandle_t tx_semaphore;
	SemaphoreHandle_t rx_semaphore;
	SemaphoreHandle_t mutex;
} I2C_Context;

typedef enum {
	I2C_DEV_1 = 0,
	I2C_DEV_2,
	I2C_DEV_4,
	I2C_DEV_COUNT
} I2C_Device;

static I2C_Context i2c_contexts[I2C_DEV_COUNT] = {0};

/**
 * @brief  Recupere un context I2C en fonction de l'instance
 * @param  dev: Instance de l'I2C
 * @retval Contexte de l'I2C
 */
static I2C_Context *I2C_GetContext(I2C_TypeDef *dev) {
	I2C_Context *ctx =NULL;

	switch ((uint32_t) dev) {
	case (uint32_t) I2C1:
			ctx= &i2c_contexts[I2C_DEV_1];
	break;
	case (uint32_t) I2C2:
			ctx= &i2c_contexts[I2C_DEV_2];
	break;
	case (uint32_t) I2C4:
			ctx= &i2c_contexts[I2C_DEV_4];
	break;
	default:
		break;
	}

	return ctx;
}

/**
 * @brief  Initialise le driver I2C
 * @param  dev: Instance de l'I2C
 * @retval HAL_OK si l'initialisation s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Init(I2C_TypeDef *dev) {
	assert_param(dev != NULL);

	I2C_Context *ctx = I2C_GetContext(dev);
	assert_param(ctx != NULL);

	ctx->tx_semaphore = xSemaphoreCreateBinary();
	assert_param(ctx->tx_semaphore != NULL);

	ctx->tx_semaphore = xSemaphoreCreateBinary();
	assert_param(ctx->tx_semaphore != NULL);

	ctx->mutex = xSemaphoreCreateMutex();
	assert_param(ctx->mutex != NULL);

	switch ((uint32_t)dev) {
	case (uint32_t)I2C1:
			MX_I2C1_Init();
	ctx->hi2c = &hi2c1;

	vQueueAddToRegistry(ctx->tx_semaphore, "SEM I2C1 TX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C1 RX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C1 Mutex");
	break;
	case (uint32_t)I2C2:
			MX_I2C2_Init();
	ctx->hi2c = &hi2c2;

	vQueueAddToRegistry(ctx->tx_semaphore, "SEM I2C2 TX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C2 RX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C2 Mutex");
	break;
	case (uint32_t)I2C4:
			MX_I2C4_Init();
	ctx->hi2c = &hi2c4;

	vQueueAddToRegistry(ctx->tx_semaphore, "SEM I2C4 TX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C4 RX");
	vQueueAddToRegistry(ctx->rx_semaphore, "SEM I2C4 Mutex");
	break;
	default:
		return HAL_ERROR;
	}

	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_MASTER_TX_COMPLETE_CB_ID, onTXEvent);
	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_MASTER_RX_COMPLETE_CB_ID, onRXEvent);
	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_ERROR_CB_ID, onErrorEvent);
	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_ABORT_CB_ID, onAbortEvent);
	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_MEM_TX_COMPLETE_CB_ID, onTXEvent);
	HAL_I2C_RegisterCallback(ctx->hi2c, HAL_I2C_MEM_RX_COMPLETE_CB_ID, onRXEvent);

	return HAL_OK;
}

/**
 * @brief  Envoi de données sur l'I2C
 * @param  dev: Instance de l'I2C
 * @param  addr: Adresse de l'esclave I2C
 * @param  pData: Données à envoyer
 * @param  size: Taille des données à envoyer
 * @param  timeout: Timeout d'attente (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Transmit(I2C_TypeDef *dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout) {
	assert_param(dev != NULL);
	assert_param(pData != NULL);
	assert_param(size > 0);
	assert_param(timeout != 0);

	I2C_Context *ctx = I2C_GetContext(dev);
	assert_param(ctx != NULL);

	// Prise du mutex pour éviter les accès concurrents
	if (xSemaphoreTake(ctx->mutex, timeout) != pdTRUE)
		return HAL_ERROR;

	if (HAL_I2C_Master_Transmit_IT(ctx->hi2c, addr, pData, size) != HAL_OK) {
		xSemaphoreGive(ctx->mutex); // Libération du mutex
		return HAL_ERROR;
	}

	// Attendre la fin de la transmission
	if (xSemaphoreTake(ctx->tx_semaphore, timeout) != pdTRUE) {
		xSemaphoreGive(ctx->mutex); // Libération du mutex
		return HAL_TIMEOUT;
	}

	xSemaphoreGive(ctx->mutex); // Libération du mutex
	return HAL_OK;
}

/**
 * @brief  Lecture de données sur l'I2C
 * @param  dev: Instance de l'I2C
 * @param  addr: Adresse de l'esclave I2C
 * @param  pData: Buffer pour recevoir les données lues
 * @param  size: Taille des données à lire
 * @param  timeout: Timeout d'attente (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Receive(I2C_TypeDef *dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout) {
	assert_param(dev != NULL);
	assert_param(pData != NULL);
	assert_param(size > 0);
	assert_param(timeout != 0);

	I2C_Context *ctx = I2C_GetContext(dev);
	assert_param(ctx != NULL);

	// Prise du mutex pour éviter les accès concurrents
	if (xSemaphoreTake(ctx->mutex, timeout) != pdTRUE)
		return HAL_ERROR;

	if (HAL_I2C_Master_Receive_IT(ctx->hi2c, addr, pData, size) != HAL_OK) {
		xSemaphoreGive(ctx->mutex); // Libération du mutex
		return HAL_ERROR;
	}

	// Attendre la fin de la réception
	if (xSemaphoreTake(ctx->rx_semaphore, timeout) != pdTRUE) {
		xSemaphoreGive(ctx->mutex); // Libération du mutex
		return HAL_TIMEOUT;
	}

	xSemaphoreGive(ctx->mutex); // Libération du mutex
	return HAL_OK;
}

/**
 * @brief  Fonction de callback de fin de transmission de l'I2C. Appelée à chaque fois que le transfert I2C est terminé
 * @param  hi2c: Handle de l'I2C
 * @retval None
 */
static void onTXEvent(I2C_HandleTypeDef *hi2c) {
	assert_param(hi2c != NULL);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	I2C_Context *ctx = I2C_GetContext(hi2c->Instance);
	assert_param(ctx != NULL);

	// Libération du sémaphore de transmission
	xSemaphoreGiveFromISR(ctx->tx_semaphore, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The
		actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  Fonction de callback de fin de réception de l'I2C. Appelée à chaque fois que la reception des données est finie
 * @param  hi2c: Handle de l'I2C
 * @retval None
 */
static void onRXEvent(I2C_HandleTypeDef *hi2c) {
	assert_param(hi2c != NULL);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	I2C_Context *ctx = I2C_GetContext(hi2c->Instance);
	assert_param(ctx != NULL);

	// Libération du sémaphore de reception
	xSemaphoreGiveFromISR(ctx->rx_semaphore, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The
			actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  Fonction de callback d'erreur de l'I2C. Appelée à chaque fois qu'une erreur I2C se produit
 * @param  hi2c: Handle de l'I2C
 * @retval None
 */
static void onErrorEvent(I2C_HandleTypeDef *hi2c) {
	assert_param(0); // TODO: Implement error handling
}

/**
 * @brief  Fonction de callback d'erreur d'abort de l'I2C. Appelée à chaque fois qu'une erreur d'abort se produit
 * @param  hi2c: Handle de l'I2C
 * @retval None
 */
static void onAbortEvent(I2C_HandleTypeDef *hi2c) {
	assert_param(0); // TODO: Implement abort error handling
}

