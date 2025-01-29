/*
 * debug.c
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */
#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "app_config.h"

#include <cstdarg> // Pour gérer les arguments variables
#include <cstdio>  // Pour printf

// ITM Stimulus Port pour SWO
#define ITM_STIMULUS_PORT_PRINTF 			0
#define ITM_STIMULUS_PORT_PERIODIC_DEBUG 	1

void Debug::init(void) {
#ifdef DEBUG
	// Creation de la tache de rapport periodique
	xTaskCreate(periodicReportTask_,
			"DebugPeriodic",
			TASK_STACK_SIZE_STD*4,
			NULL,
			TASK_PRIO_PERIODIC_DEBUG,
			&periodicReportTaskHandle_);

	vTaskResume(periodicReportTaskHandle_);
#endif // DEBUG
}


void Debug::write_(uint8_t port, const char c) {
#ifdef DEBUG
	assert_param(port<=DEBUG_VCP_PORT);
	if (port!=DEBUG_VCP_PORT) {
		if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
			while (ITM->PORT[port].u32 == 0) {} // Attend que le port soit prêt
			ITM->PORT[port].u8 = (uint8_t)c;   // Écrit un caractère
		}
	} else {
		// envoi sur l'UART connectée à la sonde debug
		// [TODO] a reprendre
		//HAL_UART_Transmit(huart, pData, Size, Timeout);
	}
#endif // DEBUG
}

void Debug::write_(uint8_t port, const char *str) {
#ifdef DEBUG
	assert_param(port<=DEBUG_VCP_PORT);

	if (port!=DEBUG_VCP_PORT) {
		if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
			while (*str!=0) {
				while (ITM->PORT[port].u32 == 0) {} // Attend que le port soit prêt
				ITM->PORT[port].u8 = (uint8_t)*str;   // Écrit un caractère

				str++;
			}
		}
	} else {
		// envoi sur l'UART connectée à la sonde debug
		// [TODO] a reprendre
		//HAL_UART_Transmit(huart, pData, Size, Timeout);
	}
#endif // DEBUG
}

void Debug::write(uint8_t port, const char* fmt, ...) {
#ifdef DEBUG
	char str[100];

	va_list args;             // Déclare une liste d'arguments
	va_start(args, fmt);   // Initialise la liste avec le dernier argument fixe
	vsnprintf(str, 100, fmt, args);    // Appelle vsnprintf avec la liste d'arguments
	va_end(args);             // Libère les ressources associées à va_list

	write_(port,(const char*)str);
#endif //DEBUG
}

void Debug::write(const char* fmt, ...) {
#ifdef DEBUG
	char str[100];

	va_list args;             // Déclare une liste d'arguments
	va_start(args, fmt);   // Initialise la liste avec le dernier argument fixe
	vsnprintf(str, 100, fmt, args);    // Appelle vsnprintf avec la liste d'arguments
	va_end(args);             // Libère les ressources associées à va_list

	write_(DEBUG_DEFAULT_PORT,(const char*)str);
#endif //DEBUG
}

void Debug::periodicReportTask_(void *pvParameters) {
#ifdef DEBUG
	char buffer[512];

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1s
		vTaskList(buffer); // Collecte les stats

		writeln(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"Task\tState\tPrio\tStack\tNum");
		writeln(ITM_STIMULUS_PORT_PERIODIC_DEBUG,buffer);
	}
#endif //DEBUG
}

int __io_putchar(int ch) {
#ifdef DEBUG
	return (int)ITM_SendChar(ch);
#else
	return ch;
#endif //DEBUG
}

int __io_getchar(void) {
	return 0;
}
