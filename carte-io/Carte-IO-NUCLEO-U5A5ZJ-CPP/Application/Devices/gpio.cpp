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
Gpio::Gpio(const char* taskName) : Sensor() {
	Debug::writeln("[Gpio] Creation de l'objet Gpio");

	// Création de la tache associée à la méthode run()
	if (!taskHandler_.create([&](void) { run(); },
			taskName,
			TASK_STACK_SIZE_STD,
			TASK_PRIO_GPIO)) {
		PANIC("[Gpio] Unable to create task run()");
	}
}

// Destructeur
Gpio::~Gpio() {
	Debug::writeln("[Gpio] Destruction de l'objet Gpio");
}

// Méthode de la classe appelée par la tâche taskWrapper
void Gpio::run() {
	Message *msg;
	LogMessage *ans;
	GpioMessage *gpioMsg;

	int counter=0;

	Debug::writeln("[Gpio] Démarrage de la tache");

	while (1) {
		/*
		 * Récupération des messages en provenance de l'application
		 * On a joute un timeout de 1s pour éviter d'etre bloquant
		 * et pouvoir, périodiquement, verifier qu'un signal en entrée a changé
		 *
		 * Si msg == nullptr -> sortie du au timeout
		 * Sinon, sortie du à la reception d'un message
		 */
		msg = mailbox_.get(1000);

		if (msg) { // on arrive ici car on a reçu un message venant de l'application
			gpioMsg = static_cast<GpioMessage*>(msg);
			ans=new LogMessage();

			if (gpioMsg->isValid()) {
				Debug::writeln("[Gpio] Reception d'un message valide");
				ans->setString("Ok");
			} else {
				Debug::writeln("[Gpio] Reception d'un message invalide");
				ans->setString("Err");
			}

			postMessageToApp(ans);

			delete (msg);
		}

		// Dans tout les cas, on scrute l'etat des ports en entrée
		// (soit toutes les 100ms si aucun message n'est reçu, sinon lorsqu'un message est reçu)

		//[TODO]: faire la scrutation des ports: si ça change, envoi d'un message
		Debug::write("[Gpio] Compteur = %d\n", counter++);
	}
}

//// Wrapper statique pour appeler la méthode membre
//void Gpio::taskWrapper(void* parameter) {
//	Gpio* instance = static_cast<Gpio*>(parameter);
//	if (instance) {
//		instance->run();
//	}
//
//	vTaskDelete(nullptr);  // Supprime la tâche si jamais la méthode run() retourne
//}

/***
 * GpioMessage
 */
GpioMessage::GpioMessage() : Message() {
	messageID_ = MESSAGE_SET_GPIO;
}

GpioMessage::GpioMessage(GPIOPins_TypeDef pins) : Message(), pins_(pins) {
	messageID_ = MESSAGE_SET_GPIO;
}

void GpioMessage::setPins (GpioMessage::GPIOPins_TypeDef pins) {
	pins_= pins;
}

GpioMessage::GPIOPins_TypeDef GpioMessage::getPins(void) {
	return pins_;
}

GpioMessage* GpioMessage::copy() {
	return new GpioMessage(pins_);
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
