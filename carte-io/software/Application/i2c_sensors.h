/*
 * i2C_sensors.h
 *
 *  Created on: May 19, 2025
 *      Author: dimercur
 */

#ifndef I2C_SENSORS_H_
#define I2C_SENSORS_H_

#include "FreeRTOS.h"
#include "queue.h"

/**
 * @brief  Fonction d'initialisation des capteurs I2C
 * @retval None
 */
void I2C_SensorsInit(QueueHandle_t *AppMsgQueue);

/**
 * @brief  Retourne le handle de la file de messages I2C
 * @retval handle de la file de messages I2C
 */
QueueHandle_t* I2C_Sensors_GetMessageQueue(void);

/**
 * @brief  Fonction pour lancer la recherche de périphériques I2C
 * @retval None
 */
void I2C_Sensors_Probe(void);

#endif /* I2C_SENSORS_H_ */
