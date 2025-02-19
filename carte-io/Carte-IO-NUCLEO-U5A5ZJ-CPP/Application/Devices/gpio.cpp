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

GpioMessage* Gpio::fromFrame(Sensor::RawData_Typedef raw) {
	GpioMessage* msg = nullptr;

	if (raw.length >=2) {
		// Seule 2 trame peuvent venir du raspberry : MESSAGE_SET_GPIO et MESSAGE_GET_GPIO_REQ
		switch (raw.data[0]) {
		case MESSAGE_SET_GPIO:
			if (raw.length ==3) {
				msg = new GpioMessage();
				msg->setID(MESSAGE_SET_GPIO);
				msg->setPins({
					static_cast<uint8_t>(raw.data[1]& 0x0F),
					static_cast<uint8_t>(raw.data[2]& 0x0F)});
			}
			break;
		case MESSAGE_GET_GPIO_REQ:
			if (raw.length == 2) {
				msg = new GpioMessage();
				msg->setID(MESSAGE_GET_GPIO_REQ);

				// Sur un message Request seules les pins à lire nous intéressent
				msg->setPins({static_cast<uint8_t>(raw.data[1]& 0xF0),0});
			}
			break;
		default:
			break;
		}
	}

	return msg;
}

Sensor::RawData_Typedef Gpio::toFrame(GpioMessage* msg) {
	Sensor::RawData_Typedef raw = {0, nullptr};

	// le seul message à renvoyer est MESSAGE_GET_GPIO_ANS
	if (msg->getID() == MESSAGE_GET_GPIO_ANS ) {
		raw.length = 2; // 1 octet pour le type (MESSAGE_GET_GPIO_ANS), 1 pour les valeurs
		raw.data = static_cast<uint8_t*>(malloc(raw.length));

		raw.data[0] = MESSAGE_GET_GPIO_ANS;
		// Sur un message Request seules les valeurs des pins à lire nous intéressent
		raw.data[1] = static_cast<uint8_t>(msg->getPins().vals & 0xF0);
	}

	return raw;
}

// Méthode de la classe appelée par la tâche taskWrapper
void Gpio::run() {
	Message *msg=nullptr;
	GpioMessage *req=nullptr, *ans=nullptr;

	Debug::writeln("[Gpio] Demarrage de la tache");

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
			req=static_cast<GpioMessage*>(msg);
			ans=nullptr;

			if (req->isValid()) {
				Debug::writeln("[Gpio] Reception d'un message valide");

				switch (req->getID()) {
				case MESSAGE_SET_GPIO:
					// TODO : Choses a faire ici, mais flemme
					break;
				case MESSAGE_GET_GPIO_REQ: {
					ans = new GpioMessage();
					ans->setID(MESSAGE_GET_GPIO_ANS);
					ans->setPins({0,0xC0}); //TODO: valeurs bidons, à récupérer des pins
					break;
				}
				default:
					// message invalide, on drop
					break;
				}
			} else
				Debug::writeln("[Gpio] Reception d'un message invalide");

			if (ans)
				postMessageToApp(ans);

			delete (msg);
		}

		// Dans tout les cas, on scrute l'etat des ports en entrée
		// (soit toutes les 100ms si aucun message n'est reçu, sinon lorsqu'un message est reçu)

		//[TODO]: faire la scrutation des ports: si ça change, envoi d'un message
		//Debug::write("[Gpio] Compteur = %d\n", counter++);
	}
}

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

std::string GpioMessage::toString() {
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
