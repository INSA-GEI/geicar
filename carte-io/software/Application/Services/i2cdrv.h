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

typedef enum {
    I2C_DEV_1 = 0,
    I2C_DEV_2,
    I2C_DEV_4,
    I2C_DEV_COUNT
} I2C_Device;

// Ajout d’un type de callback erreur
typedef void (*I2C_ErrorHook)(I2C_Device dev, HAL_StatusTypeDef error);

HAL_StatusTypeDef I2C_Driver_Init(I2C_Device dev);
HAL_StatusTypeDef I2C_Driver_Transmit(I2C_Device dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout);
HAL_StatusTypeDef I2C_Driver_Receive(I2C_Device dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout);
HAL_StatusTypeDef I2C_Driver_TransmitReceive(I2C_Device dev, uint16_t addr, uint8_t *pTxData, uint16_t txSize, uint8_t *pRxData, uint16_t rxSize, TickType_t timeout);

// À appeler depuis les callbacks IRQ HAL
void I2C_Driver_TxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C_Driver_RxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C_Driver_TxRxCpltCallback(I2C_HandleTypeDef *hi2c);

// Ajout d’une fonction pour enregistrer un hook
void I2C_Driver_RegisterErrorHook(I2C_Device dev, I2C_ErrorHook hook);

#endif /* _I2CDRV_H_ */
