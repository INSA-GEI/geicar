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
//#include "stdio.h"

// ITM Stimulus Port pour SWO
#define ITM_STIMULUS_PORT_PRINTF 			0
#define ITM_STIMULUS_PORT_PERIODIC_DEBUG 	1

extern int deltaMallocFree;
char str_[100];
char buffer_[512];

Debug::Debug() {
#ifdef DEBUG
	// Création de la tache associée à la méthode run()
	if (!periodicReportTaskHandler_.create([&](void) { periodicReportTask_(); },
			"DEBUG",
			TASK_STACK_SIZE_DEBUG + configMINIMAL_STACK_SIZE,
			TASK_PRIO_PERIODIC_DEBUG)) {
		PANIC("[DEBUG] Unable to create task periodicReportTask_()");
	}
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
	va_list args;             // Déclare une liste d'arguments
	va_start(args, fmt);   // Initialise la liste avec le dernier argument fixe
	vsnprintf(str_, 100, fmt, args);    // Appelle vsnprintf avec la liste d'arguments
	va_end(args);             // Libère les ressources associées à va_list

	write_(port,(const char*)str_);
#endif //DEBUG
}

void Debug::write(const char* fmt, ...) {
#ifdef DEBUG
	//char str[100];

	va_list args;             // Déclare une liste d'arguments
	va_start(args, fmt);   // Initialise la liste avec le dernier argument fixe
	vsnprintf(str_, 100, fmt, args);    // Appelle vsnprintf avec la liste d'arguments
	va_end(args);             // Libère les ressources associées à va_list

	write_(DEBUG_DEFAULT_PORT,(const char*)str_);
#endif //DEBUG
}

void Debug::panic(const char* file, uint32_t line, const char* msg) {
	writeln("!!! PANIC - SYSTEM HALTED !!!");
	write("File: %s, line: %lu\n", file, line);
	write("%s\n", msg);

	while(1);
}

void Debug::resetDeltaMallocFree() {
	deltaMallocFree=0;
}

void Debug::periodicReportTask_(void) {
#ifdef DEBUG
	//char buffer[512]={0};

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1s
		vTaskList(buffer_); // Collecte les stats
		//
		writeln(ITM_STIMULUS_PORT_PERIODIC_DEBUG,"==========================================\nTask\tState\tPrio\tStack\tNum");
		writeln(ITM_STIMULUS_PORT_PERIODIC_DEBUG,buffer_);
		snprintf(buffer_, 512, "Delta malloc/free = %d\n",deltaMallocFree);
		writeln(ITM_STIMULUS_PORT_PERIODIC_DEBUG, buffer_);
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

