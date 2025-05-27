/*
 * debug.c
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */
#include "stm32u5xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "config.h"

#include <stdio.h>

// ITM Stimulus Port pour SWO
#define ITM_STIMULUS_PORT_PRINTF 			0
#define ITM_STIMULUS_PORT_PERIODIC_DEBUG 	1

extern uint32_t Counter_Malloc;
extern uint32_t Counter_Free;

char DEBUG_buffer[DEBUG_BUFFER_SIZE];

void vDebugperiodicTask(void *pvParameters);
TaskHandle_t DEBUG_PeriodicTaskhandle;

void DEBUG_Init(void) {
	xTaskCreate(vDebugperiodicTask,
			"DebugPeriodic",
			TASK_STACK_DEBUG,
			NULL,
			TASK_PRIO_PERIODIC_DEBUG,
			&DEBUG_PeriodicTaskhandle);

	vTaskResume(DEBUG_PeriodicTaskhandle);
}

int __io_putchar(int ch) {
	return (int)ITM_SendChar(ch);
}

int __io_getchar(void) {
	return 0;
}

void DEBUG_PrintITM(uint8_t port, char *str) {
	if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
		while (*str!=0) {
			while (ITM->PORT[port].u32 == 0) {} // Attend que le port soit prêt
			ITM->PORT[port].u8 = (uint8_t)*str;   // Écrit un caractère

			str++;
		}
	}
}

void vDebugperiodicTask(void *pvParameters) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(DEBUG_PERIODIC_TASK_DELAY);

	// Initialise xLastWakeTime avec le temps courant pour que vTaskDelayUntil fonctionne correctement
	xLastWakeTime = xTaskGetTickCount();

	for(;;) {
		// Attend la prochaine période, maintient une exécution à intervalle fixe
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		vTaskList(DEBUG_buffer); // Collecte les stats
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"Task\tState\tPrio\tStack\tNum\n");
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,DEBUG_buffer);
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"\n");

		snprintf(DEBUG_buffer,DEBUG_BUFFER_SIZE-1, "Mallocs: %lu\nFrees: %lu\nDelta: %lu\n\n", Counter_Malloc,Counter_Free, Counter_Malloc-Counter_Free);
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG, DEBUG_buffer);

		snprintf(DEBUG_buffer, DEBUG_BUFFER_SIZE-1, "%u\n", xPortGetFreeHeapSize());
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG, DEBUG_buffer);
	}
}
