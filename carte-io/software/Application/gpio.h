/*
 * gpio.h
 *
 *  Created on: May 22, 2025
 *      Author: dimercur
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "messages.h"

/**
 * @brief  Fonction d'initialisation des GPIO
 * @param  AppMsgQueue: Pointeur vers la file de messages de l'application
 * @retval None
 */
void GPIO_Init(QueueHandle_t *AppMsgQueue);

/**
 * @brief  Fonction de traitement des messages pour les GPIO
 * @param  msg: Pointeur vers le message à traiter
 * @retval HAL_StatusTypeDef: Statut de la fonction
 */
HAL_StatusTypeDef GPIO_MessageProcessor(Messages_TypeDef *msg);

#endif /* GPIO_H_ */
