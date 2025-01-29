/*
 * appwrapper.cpp
 *
 *  Created on: Jan 15, 2025
 *      Author: dimercur
 */

#include <appwrapper.h>
#include <app.h>
#include "cmsis_os2.h"

// Fonction passerelle appelée depuis le C
extern "C" void MX_FREERTOS_Init(void);

App *app = nullptr;

extern "C" __attribute__((noreturn)) void cpp_main(void) {
	/* Init scheduler */
	osKernelInitialize();

	/* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();

	// Creation de l'objet app qui sert d'objet principal de l'ensemble de l'application
	app = new App("Application", "App_Queue");

	/* Start scheduler */
	osKernelStart();

	/* cet appel ne sera jamais atteint mais sert à eviter que le destructeur de app soit appelé */
	app->run();

	__builtin_unreachable();
}
