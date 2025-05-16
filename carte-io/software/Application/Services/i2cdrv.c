/*
 * i2cdrv.c
 *
 *  Created on: May 16, 2025
 *      Author: dimercur
 */

#include "i2cdrv.h"
#include <string.h>

// Externes déclarés dans le projet
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;

extern void MX_I2C1_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_I2C4_Init(void);

// TODO: Revoir pour utiliser les fonction natives Freertos plutôt que cmsis_os2.h

typedef struct {
    I2C_HandleTypeDef *hi2c;
    osSemaphoreId_t txSem;
    osSemaphoreId_t rxSem;
    osSemaphoreId_t txrxSem;
    osMutexId_t mutex;
    I2C_ErrorHook errorHook;
} I2C_Context;

static I2C_Context i2c_contexts[I2C_DEV_COUNT] = {0};

static void create_semaphores(I2C_Context *ctx) {
    ctx->txSem = osSemaphoreNew(1, 0, NULL);
    ctx->rxSem = osSemaphoreNew(1, 0, NULL);
    ctx->txrxSem = osSemaphoreNew(1, 0, NULL);
    ctx->mutex = osMutexNew(NULL);
}

HAL_StatusTypeDef I2C_Driver_Init(I2C_Device dev) {
    I2C_Context *ctx = &i2c_contexts[dev];

    // TODO: Revoir pour utiliser les RegisterCallback

    switch (dev) {
        case I2C_DEV_1:
            MX_I2C1_Init();
            ctx->hi2c = &hi2c1;
            break;
        case I2C_DEV_2:
            MX_I2C2_Init();
            ctx->hi2c = &hi2c2;
            break;
        case I2C_DEV_4:
            MX_I2C4_Init();
            ctx->hi2c = &hi2c4;
            break;
        default:
            return HAL_ERROR;
    }

    create_semaphores(ctx);
    return HAL_OK;
}

static HAL_StatusTypeDef lock_and_start(I2C_Context *ctx, HAL_StatusTypeDef (*startFn)(void), osSemaphoreId_t sem, TickType_t timeout) {
    if (osMutexAcquire(ctx->mutex, timeout) != osOK)
        return HAL_ERROR;

    HAL_StatusTypeDef status = startFn();

    if (status != HAL_OK) {
        if (ctx->errorHook)
            ctx->errorHook((I2C_Device)(ctx - i2c_contexts), status);
        osMutexRelease(ctx->mutex);
        return status;
    }

    if (osSemaphoreAcquire(sem, timeout) != osOK) {
        if (ctx->errorHook)
            ctx->errorHook((I2C_Device)(ctx - i2c_contexts), HAL_TIMEOUT);
        osMutexRelease(ctx->mutex);
        return HAL_TIMEOUT;
    }

    osMutexRelease(ctx->mutex);
    return HAL_OK;
}


HAL_StatusTypeDef I2C_Driver_Transmit(I2C_Device dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout) {
    I2C_Context *ctx = &i2c_contexts[dev];

    /* TODO : Gerer le mutex pour la transmission */
	if (osMutexAcquire(ctx->mutex, timeout) != osOK)
		return HAL_ERROR;

    if (HAL_I2C_Master_Transmit_IT(ctx->hi2c, addr, pData, size) != HAL_OK) {
        return HAL_ERROR;
    }

    if (osSemaphoreAcquire(ctx->txSem, timeout) != osOK) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

HAL_StatusTypeDef I2C_Driver_Receive(I2C_Device dev, uint16_t addr, uint8_t *pData, uint16_t size, TickType_t timeout) {
    I2C_Context *ctx = &i2c_contexts[dev];

    /* TODO : Gerer le mutex pour la reception */
    if (osMutexAcquire(ctx->mutex, timeout) != osOK)
    	return HAL_ERROR;

    if (HAL_I2C_Master_Receive_IT(ctx->hi2c, addr, pData, size) != HAL_OK) {
        return HAL_ERROR;
    }

    if (osSemaphoreAcquire(ctx->rxSem, timeout) != osOK) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

HAL_StatusTypeDef I2C_Driver_TransmitReceive(I2C_Device dev, uint16_t addr, uint8_t *pTxData, uint16_t txSize, uint8_t *pRxData, uint16_t rxSize, TickType_t timeout) {
    I2C_Context *ctx = &i2c_contexts[dev];

    /* TODO : Gerer le mutex pour la transmission */
    if (osMutexAcquire(ctx->mutex, timeout) != osOK)
    	return HAL_ERROR;

    // TODO: revoir l'envoi suivi de reception
    if (HAL_I2C_Master_Seq_Transmit_IT(ctx->hi2c, addr, pTxData, txSize, I2C_FIRST_FRAME) != HAL_OK) {
        return HAL_ERROR;
    }

    // Attendre fin de la transmission
    if (osSemaphoreAcquire(ctx->txrxSem, timeout) != osOK) {
        return HAL_TIMEOUT;
    }

    if (HAL_I2C_Master_Seq_Receive_IT(ctx->hi2c, addr, pRxData, rxSize, I2C_LAST_FRAME) != HAL_OK) {
        return HAL_ERROR;
    }

    // Attendre fin de la réception
    if (osSemaphoreAcquire(ctx->txrxSem, timeout) != osOK) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

void I2C_Driver_RegisterErrorHook(I2C_Device dev, I2C_ErrorHook hook) {
    if (dev < I2C_DEV_COUNT) {
        i2c_contexts[dev].errorHook = hook;
    }
}

void I2C_Driver_TxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for (int i = 0; i < I2C_DEV_COUNT; ++i) {
        if (i2c_contexts[i].hi2c == hi2c) {
            osSemaphoreRelease(i2c_contexts[i].txSem);
            break;
        }
    }
}

void I2C_Driver_RxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for (int i = 0; i < I2C_DEV_COUNT; ++i) {
        if (i2c_contexts[i].hi2c == hi2c) {
            osSemaphoreRelease(i2c_contexts[i].rxSem);
            break;
        }
    }
}

void I2C_Driver_TxRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for (int i = 0; i < I2C_DEV_COUNT; ++i) {
        if (i2c_contexts[i].hi2c == hi2c) {
            osSemaphoreRelease(i2c_contexts[i].txrxSem);
            break;
        }
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Driver_TxCpltCallback(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Driver_RxCpltCallback(hi2c);
}

void HAL_I2C_MasterSeqTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Driver_TxRxCpltCallback(hi2c);
}

void HAL_I2C_MasterSeqRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    I2C_Driver_TxRxCpltCallback(hi2c);
}
