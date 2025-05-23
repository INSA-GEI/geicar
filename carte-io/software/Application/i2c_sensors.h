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


typedef struct {
	uint32_t internalSensors; // Liste des capteurs internes;
	uint32_t externalSensors[8]; // Liste des capteurs externes [8 voies], 32 capteurs possibles par voie;
} I2C_Sensor_ProbeResults_TypeDef;
/**
 * @brief  Fonction pour lancer la recherche de périphériques I2C
 * @retval None
 */
I2C_Sensor_ProbeResults_TypeDef I2C_Sensors_Probe(void);

#endif /* I2C_SENSORS_H_ */
