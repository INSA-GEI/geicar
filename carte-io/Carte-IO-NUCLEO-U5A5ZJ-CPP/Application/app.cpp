/*
 * App.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#include <app.h>
#include "app_config.h"
#include "debug.h"

#include "messages.h"

/* Constantes */
#define SOF 0x7F                  // Start of Frame
#define HEADER_SIZE 2             // Taille de [SOF][Length][Type]

App* appInstance;

App::App(const char* taskName, const char* queueName) {
	Debug::writeln("[App] Creation de l'objet App");

	// Création de la tache associée à la methode mainTask
	if (!mainTask_.create(
			[&](void){ mainTask(); },
			taskName,
			TASK_STACK_SIZE_APPLICATION,
			TASK_PRIO_APP_MAIN_TASK)) {
		while (1);
	}

	// Création de la tache associée à la methode receiveFrameTask
	if (!receiveCommandTask_.create(
			[&](void){ receiveFrameTask(); },
			"APP_receiveFrame",
			TASK_STACK_SIZE_STD,
			TASK_PRIO_APP_RCV_CMD)) {
		while (1);
	}

	/* Creation de la message queue principale de app */
	if (!messageQueue_.create(queueName)) {
		Debug::writeln("[App] Erreur de creation de la file");
		while (1);
	}

	// Crée les sémaphores FreeRTOS
	txCompleteSemaphore_ = xSemaphoreCreateBinary();
	rxCompleteSemaphore_ = xSemaphoreCreateBinary();

	// Configure l'UART
	huart_.Init.BaudRate = 115200;
	huart_.Init.WordLength = UART_WORDLENGTH_8B;
	huart_.Init.StopBits = UART_STOPBITS_1;
	huart_.Init.Parity = UART_PARITY_NONE;
	huart_.Init.Mode = UART_MODE_TX_RX;
	huart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart_.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart_) != HAL_OK) {
		while (1);
	}

	// Enregistre les callbacks personnalisés
	if (HAL_UART_RegisterCallback(&huart_, HAL_UART_TX_COMPLETE_CB_ID, txCompleteCallback_) != HAL_OK) {
		while (1);
	}

	if (HAL_UART_RegisterCallback(&huart_, HAL_UART_RX_COMPLETE_CB_ID, rxCompleteCallback_) != HAL_OK) {
		while (1);
	}
}

App::~App() {
	Debug::writeln("[App] Destruction de l'objet App");

	vSemaphoreDelete(txCompleteSemaphore_);
	vSemaphoreDelete(rxCompleteSemaphore_);
	HAL_UART_DeInit(&huart_);

	//delete (messageQueue_);
	//delete (&mainTask_);
	//delete (&receiveCommandTask_);
}

/*
 * cette methode ne sert un peu à rien vu que les taches demarre immediatement lorsque le
 * scheduler de freertos demarre.
 * En fait, elle sert à eviter que le destructeur de app soit appelé (voir appwrapper.c)
 */
void App::run(void) {
	mainTask_.run();
}

// Méthode de la classe appelée par la tâche mainTask_
void App::mainTask(void) {
	Debug::writeln("[App] Demarrage de la tache mainTask");

	if (!appInstance)
		appInstance = this;

	gpio = new Gpio("Gpio_Tsk", "Gpio_Queue");

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));  // Attendre 1 seconde
		Debug::writeln("[App] activation de la tache");
	}
}

// Méthode appelée par la tache receiveCommandTask_
void App::receiveFrameTask(void) {
	Debug::writeln("[App] Demarrage de la tache receiveFrameTask");

	if (!appInstance)
		appInstance = this;

	/* Recuperer l'en tete d'un commande */
	if (HAL_UART_Receive_IT(&huart_, cmdHeader, 3) != HAL_OK) {
		// Gérer l'erreur ici
	}

	while (1) {
		// Attend la réception via le sémaphore
		if (xSemaphoreTake(rxCompleteSemaphore_, portMAX_DELAY) != pdTRUE) {
			// Gérer le timeout ici
		}

		Debug::writeln("[App] reception de donnée");
	}
}

// Méthodes d'interruption
void App::onTxCompleteCallback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(txCompleteSemaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void App::onRxCompleteCallback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(rxCompleteSemaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void App::txCompleteCallback_(UART_HandleTypeDef* huart) {
	//App* instance = static_cast<App*>(huart->pUserData);
	if (appInstance) {
		appInstance->onTxCompleteCallback();
	}
}

void App::rxCompleteCallback_(UART_HandleTypeDef* huart) {
	//App* instance = static_cast<App*>(huart->pUserData);
	if (appInstance) {
		appInstance->onRxCompleteCallback();
	}
}
