/*
 * leds.c
 *
 *  Created on: May 28, 2025
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"

#include "leds.h"
#include "sw_timer.h"

#include "main.h" // Pour les définitions GPIO de LED_ACTIVITY_Pin et LED_ACTIVITY_GPIO_Port

sw_timer_id_t ledActivityTimer;
void onTimerEvent(void *arg);
APP_State_EnumTypeDef currentActivityState = APP_STATE_INIT;

/**
 * @brief  Initialise les LEDs de l'application
 * Cette fonction configure la LED d'activité et initialise le timer pour gérer son état.
 * Elle doit être appelée au démarrage de l'application.
 */
void LEDS_Init(void) {
	// La configuration de la led Activity est déjà réalisée dans MX_GPIO_Init(), appelée dans main (main.c)

	LEDS_SetActivityState(APP_STATE_INIT); // Éteindre la LED d'activité au démarrage

	// Initialisation du timer pour la LED d'activité
	ledActivityTimer = SW_TIMER_Configure(100, onTimerEvent, NULL , SW_TIMER_PERIODIC);
}

/**
 * @brief  Définit l'état d'activité de la LED
 * @param  state: Nouvel état de l'application
 * @retval pdTRUE si l'état a été défini avec succès, pdFALSE sinon
 *
 * Cette fonction met à jour l'état d'activité de la LED en fonction de l'état de l'application.
 * Elle doit être appelée chaque fois que l'état de l'application change.
 */
BaseType_t LEDS_SetActivityState(APP_State_EnumTypeDef state) {
	assert_param(state >= APP_STATE_INIT && state <= APP_STATE_SHUTDOWN);
	BaseType_t result = pdTRUE;

	currentActivityState = state;

	return result;
}

/**
 * @brief  Fonction de callback pour le timer de la LED d'activité
 * Cette fonction est appelée périodiquement par le timer pour mettre à jour l'état de la LED d'activité
 * en fonction de l'état actuel de l'application.
 * @param  arg: Argument passé au timer (non utilisé ici)
 * @retval None
 */
void onTimerEvent(void *arg) {
	static APP_State_EnumTypeDef lastState = APP_STATE_INIT;
	static uint32_t counter = 0;

	// Vérifier si l'état a changé
	if (currentActivityState != lastState) {
		lastState = currentActivityState;
		counter = 0; // Réinitialiser le compteur si l'état change
	}

	switch (currentActivityState) {
	case APP_STATE_INIT:
	case APP_STATE_SHUTDOWN:
		counter=0;
		// Éteindre la LED d'activité
		HAL_GPIO_WritePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin,
				GPIO_PIN_RESET);
		break;
	case APP_STATE_RUNNING:
		// Clignoter la LED d'activité toutes les 1000 ms
		if (counter % 10 == 0) {
			HAL_GPIO_TogglePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin);
		}
		break;
	case APP_STATE_PROBE:
		counter=0;

		// Allumer la LED d'activité en continu
		HAL_GPIO_WritePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin,
				GPIO_PIN_SET);
		break;
	case APP_STATE_LOW_BATTERY:
		// Allumer la LED d'activité 200 ms puis l'éteindre 800 ms
		if (counter <=2)
			HAL_GPIO_WritePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin,
					GPIO_PIN_SET);
		else if (counter <=10)
			HAL_GPIO_WritePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin,
					GPIO_PIN_RESET);
		else
			counter = 0; // Réinitialiser le compteur après le cycle

		break;
	case APP_STATE_ERROR:
		// Clignoter la LED d'activité rapidement toutes les 100 ms
		if (counter % 1 == 0) {
			HAL_GPIO_TogglePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin);
		}
		break;
	default:
		// État inconnu, éteindre la LED d'activité
		HAL_GPIO_WritePin(LED_ACTIVITY_GPIO_Port, LED_ACTIVITY_Pin,
				GPIO_PIN_RESET);
		break;
	}

	counter++;
}
