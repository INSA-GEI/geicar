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

// ITM Stimulus Port pour SWO
#define ITM_STIMULUS_PORT_PRINTF 			0
#define ITM_STIMULUS_PORT_PERIODIC_DEBUG 	1

void vDebugperiodicTask(void *pvParameters);
TaskHandle_t DEBUG_PeriodicTaskhandle;

void DEBUG_Init(void) {
	xTaskCreate(vDebugperiodicTask,
			"DebugPeriodic",
			TASK_STACK_SIZE_STD*4,
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
	char buffer[512];

	for(;;) {
		vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1s
		vTaskList(buffer); // Collecte les stats

		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"Task\tState\tPrio\tStack\tNum\n");
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,buffer);
		DEBUG_PrintITM(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"\n");
	}
}
