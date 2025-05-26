/*
 * sw_timer.h
 *
 *  Created on: May 26, 2025
 *      Author: dimercur
 */

#ifndef SW_TIMER_H_
#define SW_TIMER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // Pour le mutex
#include "config.h"

// Type de l'ID d'un timer
typedef int32_t sw_timer_id_t;

// Type de la fonction de rappel (callback) d'un timer
typedef void (*sw_timer_callback_t)(void *arg);

// --- API Publique ---

typedef enum {
	SW_TIMER_ONESHOT =0 ,
	SW_TIMER_PERIODIC // Timer périodique
} SW_TimerAutoReload_t;

#define SW_TIMER_NO_TIMER_AVAILABLE -1 // Valeur de retour pour indiquer qu'aucun timer n'est disponible

/**
 * @brief Initialise le service de timers logiciels.
 * Crée la tâche FreeRTOS et initialise les ressources.
 * Appelle MX_TIM7_Init().
 * @return pdTRUE si l'initialisation a réussi, pdFALSE sinon.
 */
BaseType_t SW_TIMER_Init(void);

/**
 * @brief Configure un timer logiciel.
 * @param period_ms La période du timer en millisecondes.
 * @param callback La fonction de rappel à appeler quand le timer expire.
 * @param arg Un argument optionnel à passer à la fonction de rappel. Peut être NULL.
 * @param reload Indique si le timer est periodique (SW_TIMER_PERIODIC) ou a activation unique (SW_TIMER_ONESHOT).
 * @return timer id (>=0) si la configuration a réussi, -1 sinon.
 */
sw_timer_id_t SW_TIMER_Configure(uint32_t period_ms, sw_timer_callback_t callback, void *arg, SW_TimerAutoReload_t reload);

/**
 * @brief Démarre un timer logiciel.
 * @param timer_id L'ID du timer à démarrer.
 * @return pdTRUE si le démarrage a réussi, pdFALSE sinon (timer_id invalide ou déjà démarré).
 */
BaseType_t SW_TIMER_Start(sw_timer_id_t timer_id);

/**
 * @brief Arrête un timer logiciel.
 * @param timer_id L'ID du timer à arrêter.
 * @return pdTRUE si l'arrêt a réussi, pdFALSE sinon (timer_id invalide ou déjà arrêté).
 */
BaseType_t SW_TIMER_Stop(sw_timer_id_t timer_id);

#endif /* SW_TIMER_H_ */
