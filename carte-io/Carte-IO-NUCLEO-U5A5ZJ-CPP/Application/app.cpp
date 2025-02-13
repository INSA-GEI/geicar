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

App *appInstance;

extern UART_HandleTypeDef huart1;

App::App(const char *taskName, const char *queueName) {
	Debug::writeln("[App] Creation de l'objet App");

	// Création de la tache associée à la methode mainTask
	if (!mainTask_.create([&](void) {
		mainTask();
	},
	taskName,
	TASK_STACK_SIZE_APPLICATION,
	TASK_PRIO_APP_MAIN_TASK)) {
		PANIC("[APP] Unable to create mainTask");
	}

	// Création de la tache associée à la methode receiveFrameTask
	if (!receiveCommandTask_.create([&](void) {
		receiveFrameTask();
	},
	"APP_receiveFrame",
	TASK_STACK_SIZE_STD,
	TASK_PRIO_APP_RCV_CMD)) {
		PANIC("[APP] Unable to create receiveCommandTask");
	}

	/* Creation de la message queue principale de app */
	if (!messageQueue_.create(queueName)) {
		PANIC("[App] Erreur de creation de la file");
	}

	// Configure l'USART1 -> uart pour la communication avec la raspberry
	huart1 = {0};
	huart1.Instance = USART1;

	uartDriver_ = new UartDriver(&huart1);
	//uartDriver_->configure(115200); // sans autre info, c'est du polling
	uartDriver_->configure(115200, MODE_IRQ, MODE_IRQ); // TX et RX en IT
}

App::~App() {
	Debug::writeln("[App] Destruction de l'objet App");

	delete (uartDriver_);

	//delete (messageQueue_);
	//delete (&mainTask_);
	//delete (&receiveCommandTask_);
}

/*
 * cette méthode ne sert un peu à rien vu que les taches démarrent immédiatement lorsque le
 * scheduler de freertos démarre.
 * En fait, elle sert à éviter que le destructeur de app soit appelé (voir appwrapper.c)
 */
void App::run(void) {
	mainTask_.run();
}

// Méthode de la classe appelée par la tâche mainTask_
void App::mainTask(void) {
	Debug::writeln("[App] Démarrage de la tache mainTask");
	Message* msg;

	if (!appInstance)
		appInstance = this;

	gpio_ = new Gpio("Gpio_Tsk", "Gpio_Queue", this->messageQueue_);

	while (1) {
		//vTaskDelay(pdMS_TO_TICKS(1000));  // Attendre 1 seconde
		msg=messageQueue_.get(); //Attente infinie
		Debug::writeln("[App] Reception d'un message");

		uint8_t id= msg->getID();
		Debug::write("ID= %u",id );

		delete(msg);
		//uartDriver_->write("Hello", sizeof("Hello"));
	}
}

// Méthode appelée par la tache receiveCommandTask_
void App::receiveFrameTask(void) {
	GpioMessage* msg=nullptr;
	//QueueHandle_t queue=nullptr;
	Debug::writeln("[App] Demarrage de la tache receiveFrameTask");

	if (!appInstance)
		appInstance = this;

	/* Recuperer l'en tete d'un commande */
	//	if (HAL_UART_Receive_IT(&huart_, cmdHeader_, 3) != HAL_OK) {
	//		// Gérer l'erreur ici
	//	}



	while (1) {
		//vTaskDelay(pdMS_TO_TICKS(1000));  // Attendre 1 seconde

		if (uartDriver_->read(cmdHeader_, sizeof(cmdHeader_)) == HAL_OK ) {

			uartDriver_->write(cmdHeader_, sizeof(cmdHeader_));
			msg = new GpioMessage({cmdHeader_[0], cmdHeader_[1]});

			//queue = gpio_->getQueueHandle();
			//if (queue != nullptr) {
				if (gpio_->postMessage(msg))
				//if (xQueueSend(queue, (void*)&msg, pdMS_TO_TICKS(100)) == pdPASS)
					Debug::writeln("[App] Envoi du message OK");
				else
					Debug::writeln("[App] Echec envoi du message");
				//gpio_->postMessage(msg);
			//} else
			//	Debug::writeln("[App] Queue invalide");
		} else {
			Debug::writeln("[App] Error when receiving uart data");
		}

		Debug::writeln("[App] reception de donnée");
	}
}

