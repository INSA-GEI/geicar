/*
 * leds.h
 *
 *  Created on: May 28, 2025
 *      Author: dimercur
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "FreeRTOS.h"
#include "config.h"

/**
 * @brief  Initialise les LEDs de l'application
 * Cette fonction configure la LED d'activité et initialise le timer pour gérer son état.
 * Elle doit être appelée au démarrage de l'application.
 */
void LEDS_Init(void);

/**
 * @brief  Définit l'état d'activité de la LED
 * @param  state: Nouvel état de l'application
 * @retval pdTRUE si l'état a été défini avec succès, pdFALSE sinon
 *
 * Cette fonction met à jour l'état d'activité de la LED en fonction de l'état de l'application.
 * Elle doit être appelée chaque fois que l'état de l'application change.
 */
BaseType_t LEDS_SetActivityState(APP_State_EnumTypeDef state);

#endif /* LEDS_H_ */
