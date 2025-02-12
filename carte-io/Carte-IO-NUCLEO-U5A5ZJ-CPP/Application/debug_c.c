/*
 * debug.c
 *
 *  Created on: Feb 11, 2025
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"
#include "debug.h"
#include "stdio.h"

static void panicWriteChar(uint8_t c) {
	if (ITM->TCR & ITM_TCR_ITMENA_Msk) { // Vérifie si l'ITM est activé
		while (ITM->PORT[DEBUG_DEFAULT_PORT].u32 == 0) {} // Attend que le port soit prêt
		ITM->PORT[DEBUG_DEFAULT_PORT].u8 = (uint8_t)c;   // Écrit un caractère
	}
}

static void panicWriteStr(const char* str) {
	while (*str != 0) {
		panicWriteChar((uint8_t)*str);
		str++;
	}
}

void panic (const char *file, uint32_t line, const char *msg) {
	char buffer[20];

	panicWriteStr("Panic raised, file: ");
	panicWriteStr(file);
	panicWriteStr(", line: ");
	snprintf(buffer, 20, "%lu", line);    // Appelle snprintf pour convertir uint32_t en char*
	panicWriteStr(buffer);
	panicWriteStr(", msg: ");
	panicWriteStr(msg);
	panicWriteChar('\n');

	for (;;);
}
