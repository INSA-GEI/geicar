/*
 * sw_timer.c
 *
 *  Created on: May 26, 2025
 *      Author: dimercur
 */

#include "sw_timer.h"
#include <string.h> // Pour memset

// --- Fonctions Privées (internes au module) ---

// États possibles d'un timer
typedef enum {
	SW_TIMER_STATE_IDLE = 0,    // Non configuré
	SW_TIMER_STATE_STOPPED,     // Configuré mais arrêté
	SW_TIMER_STATE_RUNNING,      // En cours d'exécution
} SW_TimerState_t;

// Structure interne pour chaque timer
typedef struct {
	SW_TimerState_t state;           // État actuel du timer
	SW_TimerAutoReload_t autoreload;
	uint32_t period_ms;              // Période du timer en ms
	uint32_t countdown_ms;           // Compte à rebours actuel en ms
	sw_timer_callback_t callback;    // Fonction de rappel à exécuter
	void *callback_arg;              // Argument optionnel pour le callback
} SW_Timer_t;

// Déclaration de la tâche du service de timer
static void TimerService_Task(void *argument);

// Tableau des timers logiciels
static SW_Timer_t s_sw_timers[MAX_SW_TIMERS];

TaskHandle_t SW_TIMER_Taskhandle;

/**
 * @brief Fonction de la tâche FreeRTOS qui gère les timers logiciels.
 * @param argument Argument passé à la tâche (non utilisé ici).
 */
static void TimerService_Task(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(SW_TIMER_BASE_PERIOD_MS);

	// Initialise xLastWakeTime avec le temps courant pour que vTaskDelayUntil fonctionne correctement
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		// Attend la prochaine période, maintient une exécution à intervalle fixe
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Prend le mutex pour accéder en toute sécurité aux données des timers

		for (int i = 0; i < MAX_SW_TIMERS; i++) {
			if (s_sw_timers[i].state == SW_TIMER_STATE_RUNNING) {

				if ((s_sw_timers[i].countdown_ms - SW_TIMER_BASE_PERIOD_MS)<0) {
					s_sw_timers[i].countdown_ms = 0; // Timer expiré
				} else {
					s_sw_timers[i].countdown_ms -= SW_TIMER_BASE_PERIOD_MS; // Décrémente le compte à rebours
				}

				if (s_sw_timers[i].countdown_ms == 0) { // Le timer a expiré
					if (s_sw_timers[i].autoreload == SW_TIMER_ONESHOT) {
						// Arrête le timer après expiration
						s_sw_timers[i].state = SW_TIMER_STATE_STOPPED;
					}

					s_sw_timers[i].countdown_ms = s_sw_timers[i].period_ms; // Réinitialise pour une éventuelle prochaine utilisation

					// Appelle la fonction de rappel (callback)
					if (s_sw_timers[i].callback != NULL) {
						s_sw_timers[i].callback(s_sw_timers[i].callback_arg);
					}
				}
			}
		}
	}
}


// --- API Publique ---

BaseType_t SW_TIMER_Init(void) {
	// 1. Initialise les timers à l'état IDLE
	memset(s_sw_timers, 0, sizeof(s_sw_timers));
	for (int i = 0; i < MAX_SW_TIMERS; i++) {
		s_sw_timers[i].state = SW_TIMER_STATE_IDLE;
	}

	// 4. Crée la tâche du service de timer
	if (xTaskCreate(TimerService_Task,
			"SW_Timer_Service",
			TASK_STACK_SIZE_STD, // Augmente la taille de la pile si tu as beaucoup de timers ou callbacks lourds
			NULL,
			TASK_PRIO_SW_TIMERS, // Priorité supérieure à la tâche Idle
			&SW_TIMER_Taskhandle) != pdPASS) {
		// Gérer l'erreur : pas assez de mémoire pour la tâche

		return pdFALSE;
	}

	vTaskResume(SW_TIMER_Taskhandle);

	return pdTRUE;
}

sw_timer_id_t SW_TIMER_Configure(uint32_t period_ms, sw_timer_callback_t callback, void *arg, SW_TimerAutoReload_t reload) {
	if (period_ms == 0 || callback == NULL || reload > SW_TIMER_PERIODIC || reload < SW_TIMER_ONESHOT) {
		return pdFALSE; // Paramètres invalides
	}

	sw_timer_id_t p_timer_id = SW_TIMER_NO_TIMER_AVAILABLE;

	for (int i = 0; i < MAX_SW_TIMERS; i++) {
		if (s_sw_timers[i].state == SW_TIMER_STATE_IDLE) { // Trouve un emplacement libre
			s_sw_timers[i].state = SW_TIMER_STATE_STOPPED;
			s_sw_timers[i].autoreload = reload; // Définit le type de timer (auto-reload ou one-shot)
			s_sw_timers[i].period_ms = period_ms;
			s_sw_timers[i].countdown_ms = period_ms; // Initialise le compte à rebours
			s_sw_timers[i].callback = callback;
			s_sw_timers[i].callback_arg = arg;
			p_timer_id = i; // L'ID est l'index dans le tableau
			break;
		}

	}

	return p_timer_id;
}

BaseType_t SW_TIMER_Start(sw_timer_id_t timer_id) {
	BaseType_t ret = pdFALSE;

	if (timer_id < MAX_SW_TIMERS) {

		if (s_sw_timers[timer_id].state == SW_TIMER_STATE_STOPPED) {
			s_sw_timers[timer_id].state = SW_TIMER_STATE_RUNNING;
			s_sw_timers[timer_id].countdown_ms = s_sw_timers[timer_id].period_ms; // Réinitialise le compte à rebours au démarrage
			ret = pdTRUE;
		}

	}
	return ret;
}

BaseType_t SW_TIMER_Stop(sw_timer_id_t timer_id) {
	BaseType_t ret = pdFALSE;

	if (timer_id < MAX_SW_TIMERS) {
		if (s_sw_timers[timer_id].state == SW_TIMER_STATE_RUNNING) {
			s_sw_timers[timer_id].state = SW_TIMER_STATE_STOPPED;
			ret = pdTRUE;
		}
	}

	return ret;
}

