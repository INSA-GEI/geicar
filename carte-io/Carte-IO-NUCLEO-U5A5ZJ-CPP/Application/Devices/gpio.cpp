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
	//messageQueueName_ = queueName;

	// Création de la tâche FreeRTOS, la fonction statique est utilisée comme point d'entrée
	xTaskCreate(taskWrapper, taskName_, 256, this, tskIDLE_PRIORITY+1, &taskHandle_);
	vTaskResume(taskHandle_);

//	/* Creation de la message queue associée à l'objet */
//	messageQueue_ = xQueueCreate((UBaseType_t) QUEUE_LENGTH, (UBaseType_t)ITEM_SIZE);
//	if (messageQueue_ == NULL) {
//		PANIC("[GPIO] Erreur de creation de la file");
//	} else {
//		messageQueueName_ = queueName;
//		vQueueAddToRegistry(messageQueue_, messageQueueName_ );
//	}
}

// Destructeur
Gpio::~Gpio() {
	Debug::writeln("[Gpio] Destruction de l'objet Gpio");

	// Retire les semaphores et taches
	vTaskDelete(taskHandle_);
}

// Méthode de la classe appelée par la tâche taskWrapper
void Gpio::run() {
	//void* p;
	Message *msg;
	LogMessage *ans;
	GpioMessage *gpioMsg;

	int counter=0;

	Debug::writeln("[Gpio] Démarrage de la tache");

	while (1) {
		//		msg = mailbox_.get(1000); // attente sur message ou 1s
		//
		//		if (msg != nullptr ) {
		//			/* Un message a été reçu */
		//			Debug::writeln("[Gpio] Message reçu");
		//
		//			if (msg->getID()!=MESSAGE_SET_GPIO) {
		//				Debug::writeln("[Gpio] Message n'est pas de type GpioMessage");
		//			} else {
		//				gpioMsg = static_cast<GpioMessage*>(msg);
		//				GpioMessage::GPIOPins_TypeDef pins = gpioMsg->getPins();
		//
		//				Debug::write("Pins = %0xd, vals = %0xd\n", pins.pins, pins.vals);
		//			}
		//
		//			delete(msg);
		//		}

		//vTaskDelay(pdMS_TO_TICKS(1000));

		//if (xQueueReceive(messageQueue_, (void*)&msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
			//msg = static_cast<Message*>(p);

		msg = mailbox_.get(1000);
		if (msg) {
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
