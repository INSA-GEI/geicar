/*
 * Gpio.cpp
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#include <Devices/gpio.h>
#include "app_config.h"
#include "debug.h"

#include "semphr.h"

// Constructeur
Gpio::Gpio(const char* taskName, const char* queueName, MessageHandler &app_mailbox) : Sensor(queueName, app_mailbox) {
	Debug::writeln("[Gpio] Creation de l'objet Gpio");

	taskName_ = taskName;
	messageQueueName_ = queueName;

	// Création de la tâche FreeRTOS, la fonction statique est utilisée comme point d'entrée
	xTaskCreate(taskWrapper, taskName_, 256, this, tskIDLE_PRIORITY+1, &taskHandle_);
	vTaskResume(taskHandle_);
}

// Destructeur
Gpio::~Gpio() {
	Debug::writeln("[Gpio] Destruction de l'objet Gpio");

	// Retire les semaphores et taches
	vTaskDelete(taskHandle_);
}

// Méthode de la classe appelée par la tâche taskWrapper
void Gpio::run() {
	Message *msg;
	int counter=0;

	Debug::writeln("[Gpio] Démarrage de la tache");

	while (1) {
		msg = mailbox_.get(1000); // attente sur message ou 1s

		if (msg != nullptr ) {
			/* Un message a été reçu */
			Debug::writeln("[Gpio] Message reçu");
		}

		// Dans tout les cas, on scrute l'etat des ports en entrée
		// (soit toutes les 100ms si aucun message n'est reçu, sinon lorsqu'un message est reçu)

		//[TODO]: faire la scrutation des ports: si ça change, envoi d'un message
		Debug::write("[Gpio] Compteur = %d\n", counter++);
	}
}

// Wrapper statique pour appeler la méthode membre
void Gpio::taskWrapper(void* parameter) {
	Gpio* instance = static_cast<Gpio*>(parameter);
	if (instance) {
		instance->run();
	}

	vTaskDelete(nullptr);  // Supprime la tâche si jamais la méthode run() retourne
}


/***
 * GpioMessage
 */
GpioMessage::GpioMessage(MessageHandler &from, MessageHandler &to) : Message(from, to) {
	messageID_ = MESSAGE_SET_GPIO;
}

GpioMessage::GpioMessage(MessageHandler &from, MessageHandler &to, GPIOPins_TypeDef pins) : Message(from, to), pins_(pins) {
	messageID_ = MESSAGE_SET_GPIO;
}

void GpioMessage::setPins (GpioMessage::GPIOPins_TypeDef pins) {
	pins_= pins;
}

GpioMessage::GPIOPins_TypeDef GpioMessage::getPins(void) {
	return pins_;
}

GpioMessage* GpioMessage::copy() {
	return new GpioMessage(*from_, *to_, pins_);
}

std::string GpioMessage::getString() {
	return "GpioMessage";
}

/**
 * Verify if message ID is compatible with current message type
 * @param id Message ID
 * @return true, if message ID is acceptable, false otherwise
 */
bool GpioMessage::checkID(MessageID id) {
	return ((id==MESSAGE_SET_GPIO) ||
			(id==MESSAGE_GET_GPIO_ANS) ||
			(id==MESSAGE_GET_GPIO_REQ)) ? true:false;
}
