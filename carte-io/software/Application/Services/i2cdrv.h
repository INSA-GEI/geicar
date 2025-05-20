/*
 * i2cdrv.h
 *
 *  Created on: May 16, 2025
 *      Author: dimercur
 */

#ifndef _I2CDRV_H_
#define _I2CDRV_H_

#include "stm32u5xx.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"

/**
 * @brief  Initialise le driver I2C
 * @param  dev: Instance de l'I2C
 * @retval HAL_OK si l'initialisation s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Init(I2C_TypeDef *dev);

/**
 * @brief  Envoi de données sur l'I2C
 * @param  dev: Instance de l'I2C
 * @param  addr: Adresse de l'esclave I2C
 * @param  pData: Données à envoyer
 * @param  size: Taille des données à envoyer
 * @param  timeout: Timeout d'attente (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Transmit(I2C_TypeDef *dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout);

/**
 * @brief  Lecture de données sur l'I2C
 * @param  dev: Instance de l'I2C
 * @param  addr: Adresse de l'esclave I2C
 * @param  pData: Buffer pour recevoir les données lues
 * @param  size: Taille des données à lire
 * @param  timeout: Timeout d'attente (portMAX_DELAY pour une attente infinie)
 * @retval HAL_OK si l'écriture s'est bien passée, HAL_ERROR sinon
 */
HAL_StatusTypeDef I2C_Receive(I2C_TypeDef *dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout);

#endif /* _I2CDRV_H_ */
