/*
 * tasks.c
 *
 *  Created on: Aug 27, 2025
 *      Author: dimercur
 */

#include "tasks.h"
#include "configuration.h"

#include "app.h"

#include "calibrate.h"
#include "control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#if defined (__TESTS__)
#include "tests.h"
#endif /* __TESTS__ */

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_AppLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_AppLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xAppLoopTaskStack[ APPLOOP_TASK_STACK_SIZE ];
static StaticTask_t xAppLoopTaskTCB;

/* Handle vers la tâche */
static TaskHandle_t xAppLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_DebugLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_DebugLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xDebugLoopTaskStack[ DEBUGLOOP_TASK_STACK_SIZE ];
static StaticTask_t xDebugLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xDebugLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_UltrasoundLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_UltrasoundLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xUltrasoundLoopTaskStack[ USLOOP_TASK_STACK_SIZE ];
static StaticTask_t xUltrasoundLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xUltrasoundLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_ControlLoop (statique)
 * ------------------------------------------------------------------------- */
void TASKS_ControlLoop(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xControlLoopTaskStack[ CONTROLLOOP_TASK_STACK_SIZE ];
static StaticTask_t xControlLoopTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xControlLoopTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration de la tâche TASKS_CalibrationEvent (statique)
 * ------------------------------------------------------------------------- */
void TASKS_CalibrationEvent(void *argument);
/* Buffer pour la pile et le TCB */
static StackType_t xCalibrationTaskStack[ CALIBRATION_TASK_STACK_SIZE ];
static StaticTask_t xCalibrationTaskTCB;
/* Handle vers la tâche */
static TaskHandle_t xCalibrationTaskHandle = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du buffer pour la queue xAppLoopQueue
 * ------------------------------------------------------------------------- */
static uint8_t ucAppLoopQueueStorageArea[ APPLOOP_QUEUE_LENGTH * APPLOOP_QUEUE_ITEM_SIZE ];
static StaticQueue_t xAppLoopStaticQueue;
QueueHandle_t xAppLoopQueue = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du sémaphore de calibration (statique)
 * ------------------------------------------------------------------------- */
static StaticSemaphore_t xCalibrationSemaphoreBuffer;
SemaphoreHandle_t xCalibrationSemaphore = NULL;

/* -------------------------------------------------------------------------
 * Déclaration du timer pour les événements périodiques (statique)
 * ------------------------------------------------------------------------- */
static StaticTimer_t TASKS_TimerBuffer;
TimerHandle_t TASKS_TimerHandle = NULL;
void TASKS_TimerCallback(TimerHandle_t xTimer);

/*
 * @brief  Initialize tasks, queues, semaphores and timers.
 * This function creates the necessary FreeRTOS components for the application.
 * It sets up tasks, queues, semaphores, and timers used in the application.
 */
void TASKS_Init(void) {
	/* Création de la file pour l'application (statiquement) */
	xAppLoopQueue = xQueueCreateStatic(
			APPLOOP_QUEUE_LENGTH,          // nombre d’éléments
			APPLOOP_QUEUE_ITEM_SIZE,       // taille d’un élément
			ucAppLoopQueueStorageArea,    // buffer pour les données
			&xAppLoopStaticQueue          // buffer pour la structure de contrôle
	);

	if (xAppLoopQueue == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche AppLoop (statiquement) */
	xAppLoopTaskHandle = xTaskCreateStatic(
			TASKS_AppLoop,          // fonction de la tâche
			"AppLoop",             // nom (debug)
			APPLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			APPLOOP_TASK_PRIORITY,     // priorité
			xAppLoopTaskStack,         // buffer pile
			&xAppLoopTaskTCB           // buffer TCB
	);

	if (xAppLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche DebugLoop (statiquement) */
	xDebugLoopTaskHandle = xTaskCreateStatic(
			TASKS_DebugLoop,          // fonction de la tâche
			"DebugLoop",             // nom (debug)
			DEBUGLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			DEBUGLOOP_TASK_PRIORITY,     // priorité
			xDebugLoopTaskStack,         // buffer pile
			&xDebugLoopTaskTCB           // buffer TCB
	);

	if (xDebugLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche UltrasoundLoop (statiquement) */
	xUltrasoundLoopTaskHandle = xTaskCreateStatic(TASKS_UltrasoundLoop, // fonction de la tâche
			"UltrasoundLoop",             // nom (debug)
			USLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			USLOOP_TASK_PRIORITY,     // priorité
			xUltrasoundLoopTaskStack,         // buffer pile
			&xUltrasoundLoopTaskTCB           // buffer TCB
			);

	if (xUltrasoundLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();

	}

	/* Création de la tâche ControlLoop (statiquement) */
	xControlLoopTaskHandle = xTaskCreateStatic(TASKS_ControlLoop, // fonction de la tâche
			"ControlLoop",             // nom (debug)
			CONTROLLOOP_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CONTROLLOOP_TASK_PRIORITY,     // priorité
			xControlLoopTaskStack,         // buffer pile
			&xControlLoopTaskTCB           // buffer TCB
			);

	if (xControlLoopTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création de la tâche CalibrationEvent (statiquement) */
	xCalibrationTaskHandle = xTaskCreateStatic(TASKS_CalibrationEvent, // fonction de la tâche
			"CalibrationEvent",             // nom (debug)
			CALIBRATION_TASK_STACK_SIZE,   // taille pile (en mots de 32 bits)
			NULL,                  // paramètre d’entrée
			CALIBRATION_TASK_PRIORITY,     // priorité
			xCalibrationTaskStack,         // buffer pile
			&xCalibrationTaskTCB           // buffer TCB
			);

	if (xCalibrationTaskHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Création du sémaphore de calibration (statiquement) */
	xCalibrationSemaphore = xSemaphoreCreateBinaryStatic(&xCalibrationSemaphoreBuffer);
	if (xCalibrationSemaphore == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}

	/* Au démarrage, le sémaphore est "pris" */
	xSemaphoreTake(xCalibrationSemaphore, 0);

	/* Creation de un timer pour les evenements periodiques */
	TASKS_TimerHandle = xTimerCreateStatic("PeriodicTimers", pdMS_TO_TICKS(1), pdTRUE, (void *) 0, TASKS_TimerCallback, &TASKS_TimerBuffer);
	if (TASKS_TimerHandle == NULL) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}
	if (xTimerStart(TASKS_TimerHandle, 0) != pdPASS) {
		// Erreur : pas de mémoire statique ?
		Error_Handler();
	}
}

void TASKS_AppLoop(void *argument ) {
	void *pReceived = NULL;

	for(;;)
	{
#if defined (__TESTS__)
		TESTS_Run(); // Run tests if defined
#else
		/* Attente infinie d’un élément dans la queue */
		if (xQueueReceive(xAppLoopQueue, &pReceived, portMAX_DELAY) == pdPASS)
		{
			if (pReceived != NULL) {

				/* Traitement de l’élément reçu */
				// Exemple : cast et utilisation
				// MyStruct_t *msg = (MyStruct_t*) pReceived;
				APP_Run((AppMessage_typeDef*) pReceived);
			}
		}
#endif /* __TESTS__ */
	}
}

void TASKS_DebugLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(DEBUG_LOOP_PERIOD_MS);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

void TASKS_UltrasoundLoop(void *argument) {
	// no waiting time here : ultrasonic sensors measurements are started as soon as previous one are finished
	for (;;) {
		US_StartMeasurements();
	}
}

void TASKS_ControlLoop(void *argument) {
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(PERIOD_CAR_CONTROL_LOOP);

	/* Initialise la référence de temps */
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		CAR_CONTROL_Manage();

		// Wait for next control loop period
		// Time is compensated from others events that can make processing longer
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
//		if (mode == 0) {	//Calibration Mode
//			CAL_SteeringCalibration();
//			mode = 1;
//		} else {	//Control Mode
//			CAR_CONTROL_Manage(leftRearSpeed,rightRearSpeed, steeringAngle);
//		}
}

void TASKS_CalibrationEvent(void *argument) {
	// non periodic task, triggered by semaphore when calibration request is received

	for (;;) {
		// Exemple : tâche de gestion du CAN
		// Attente sur semaphore
		//vSemaphoreTake(xCalibrationSemaphore, portMAX_DELAY);

		// Suspention de la tache de control
		//vTaskSuspend(xAppLoopTaskHandle);

		// Calibration
		CAL_SteeringCalibration();
	}
}

/**
 * @brief Timer callback for periodic events.
 *
 * This function is called when the periodic timer expires.
 * It updates various periodic counters used in the application.
 */
// TODO :  a reprendre
void TASKS_TimerCallback(TimerHandle_t xTimer) {
	APP_PeriodicCountersUpdate();
}


