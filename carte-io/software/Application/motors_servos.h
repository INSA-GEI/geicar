/*
 * motors_servos.h
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#ifndef MOTORS_SERVOS_H_
#define MOTORS_SERVOS_H_

#include "stm32u5xx_hal.h"
#include "messages.h"

/**
 * @brief  Fonction d'initialisation des moteurs et servos
 * @retval None
 */
void MOTORS_SERVOS_Init(void);

/**
 * @brief  Fonction de traitement des messages pour les moteurs et servos
 * @param  msg: Pointeur vers le message à traiter
 * @retval HAL_StatusTypeDef: Statut de la fonction
 */
HAL_StatusTypeDef MOTORS_SERVOS_MessageProcessor(Messages_TypeDef *msg);

#endif /* MOTORS_SERVOS_H_ */
