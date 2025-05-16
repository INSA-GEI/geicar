/*
 * uartdrv.h
 *
 *  Created on: May 5, 2025
 *      Author: dimercur
 */

#ifndef __UARTDRV_H__
#define __UARTDRV_H__

#include <stm32u5xx_hal.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

/**
 * @brief  Configuration de l'UART
 */
typedef struct {
    uint8_t *rxCircularBuffer;           // Buffer de réception DMA circulaire
    uint16_t rxCircularBufferSize;      // Taille du buffer
    uint32_t rxPeriodicTimerDelay;
    uint32_t baudrate;
} UART_Config;

/**
 * @brief  Énumération pour la suppression du buffer après l'envoi
 */
typedef enum {
	UART_KeepBuffer=0,
	UART_DeleteBuffer
} UART_DeleteEnum;

/**
 * @brief  Handle du driver d'UART
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
} UART_Handle;

/**
 * @brief  Initialisation de l'UART, configuration en mode DMA circulaire pour la reception et DMA standard pour l'envoi
 * @param  handle: Handle de l'UART
 * @param  uart: Instance d'UART à configurer
 * @param  config: Configuration de l'UART
 * @retval HAL_OK si l'initialisation s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Init(UART_Handle *handle, USART_TypeDef *uart, UART_Config config);

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
HAL_StatusTypeDef UART_Write(UART_Handle *handle, uint8_t *data, uint16_t size, uint32_t timeout, UART_DeleteEnum deleteAfterSend);

/**
 * @brief  Lecture de données sur l'UART. Le buffer doit être alloué par l'utilisateur et la taille indiquée
 *         ainsi qu'un timeout (portMAX_DELAY pour une attente infinie)
 * @param  handle: Handle de l'UART
 * @param  buffer: Buffer de réception
 * @param  length: Taille du buffer
 * @param  timeout: Timeout d'attente du semaphore RX (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si la lecture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef UART_Read(UART_Handle *handle, uint8_t *buffer, uint16_t length, uint32_t timeout);

#endif /* __UARTDRV_H__ */
