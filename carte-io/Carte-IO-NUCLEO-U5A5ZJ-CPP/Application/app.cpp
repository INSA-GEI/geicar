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

#include <memory>
#include <stdlib.h>
#include <cstring>

/* Constantes */
#define SOF 0x7F                  // Start of Frame
#define HEADER_SIZE 3             // Taille de [SOF][Length][Type]

/**
 * Liste des handlers de périphériques pre-configurés
 */
extern UART_HandleTypeDef huart1;

App::App() {
	Debug::writeln("[App] Creation de l'objet App");

	// Création de la tache associée à la méthode mainTask
	if (!mailboxManagmentTask_.create([&](void) { mailboxManagment(); },
			"APP_MbxMgmt",
			TASK_STACK_SIZE_APPLICATION,
			TASK_PRIO_APP_MBX_MGMT_TASK)) {
		PANIC("[APP] Unable to create mainTask");
	}

	// Création de la tache associée à la méthode receiveFrameTask
	if (!commandsManagmentTask_.create([&](void) {	commandsManagment();	},
			"APP_CmdMgmt",
			TASK_STACK_SIZE_STD,
			TASK_PRIO_APP_CMD_MGMT_CMD)) {
		PANIC("[APP] Unable to create receiveCommandTask");
	}

	/* Creation de la message queue principale de app */
	if (!messageQueue_.create("Application")) {
		PANIC("[App] Erreur de creation de la file");
	}

	// Configure l'USART1 -> uart pour la communication avec la raspberry
	huart1 = {0};
	huart1.Instance = USART1;

	comRaspberry_ = new UartDriver(&huart1);
	comRaspberry_->configure(115200, MODE_IRQ, MODE_IRQ); // TX et RX en IT

	// Initialisation de la tâche périodique de debug (rapport)
	debug = new Debug();
}

App::~App() {
	Debug::writeln("[App] Destruction de l'objet App");

	delete (comRaspberry_);
}

/**
 * Cette méthode ne sert un peu à rien vu que les taches démarrent immédiatement lorsque le
 * scheduler de freertos démarre.
 *
 * En fait, elle sert à éviter que le destructeur de app soit appelé (voir appwrapper.c)
 */
void App::run(void) {
	mailboxManagmentTask_.run();
}

void App::probe() {
	//Todo : scan des périphériques et creation des objets

	//Creation du device GPIO
	gpio_ = new Gpio("GPIO");
	gpio_->initMessagesManagement("GPIO");
	gpio_->setApplicationMailbox(messageQueue_);

	/**
	 * On remet le compteur d'allocation mémoire à zero pour enlever toutes les allocations
	 * d'objet qui resteront tout au long de la vie du programme
	 */
	Debug::resetDeltaMallocFree();
}

bool App::parseFrameAndPost(RawFrameMessage *frame) {
	Message *msg=nullptr;
	Sensor::RawData_Typedef raw = {static_cast<uint16_t>(frame->getLength()-1), frame->getData()}; // length -1 car on retire le checksum
	bool status=false;

	switch (raw.data[0]) {
	case MESSAGE_SET_GPIO:
	case MESSAGE_GET_GPIO_REQ:
		msg = Gpio::fromFrame({raw.length, raw.data});
		status= gpio_->postMessage(msg);
		break;
	default:
		Debug::write("[parseFrame] Type 0x%02X invalide\n", raw.data[0]);
		break;
	}

	return status;
}

uint8_t App::computeChecksum(uint8_t* buffer, uint16_t length) {
	uint8_t checksum =0;

	for (uint16_t i=0; i<length; i++) {
		checksum += buffer[i];
	}

	checksum = 0xFF-checksum+1;

	return checksum;
}

bool App::buildFrameAndSend(Message *msg) {
	bool status=false;
	uint8_t* frame=nullptr;
	uint16_t frameLength=0;

	switch (msg->getID()) {
	case MESSAGE_GET_GPIO_ANS: {
		GpioMessage* gpiomsg = static_cast<GpioMessage*>(msg);

		// frameLength = 3 pour header +1 pour type +1 pour réponse +1 pour checksum =6
		frameLength=6;
		frame = static_cast<uint8_t*>(malloc(frameLength));
		memset(frame,0, frameLength);

		frame[0] = SOF;
		frame[HEADER_SIZE-1] = frameLength-HEADER_SIZE; // frame length - taille header
		frame[HEADER_SIZE] = static_cast<uint8_t>(MESSAGE_GET_GPIO_ANS);
		frame[HEADER_SIZE+1] = gpiomsg->getPins().vals;
		frame[HEADER_SIZE+2] = computeChecksum(frame, frameLength-1); // tout sauf le checksum

		break;
	}
	default:
		break;
	};

	if (frame)
		if (comRaspberry_->write(frame, frameLength, DELETE_BUFFER_AFTER_USE)==HAL_OK)
			status=true;

	// TODO: Problème avec le buffer alloué: comment le libérer vu que la transmission est asynchrone ?
	return status;
}

// Méthode de la classe appelée par la tâche mainTask_
void App::mailboxManagment(void) {
	Debug::writeln("[App] Demarrage de la tache mailboxManagment");
	Message* msg;

	// Scan des périphériques
	probe();

	while (1) {
		msg=messageQueue_.get(); //Attente infinie

		if (msg != nullptr) {
			Debug::writeln("[App] Reception d'un message");

			uint8_t id= msg->getID();
			Debug::write("ID= %u\n",id );

			switch (id) {
			case MESSAGE_RAW_FRAME: {
				RawFrameMessage *frame = static_cast<RawFrameMessage *>(msg);
				Debug::write("Message classe: %s\n",msg->toString().c_str());
				Debug::write("Longueur: %u\n",frame->getLength());
				Debug::write("@Data: %08X\n",reinterpret_cast<uint32_t>(frame->getData()));

				if (parseFrameAndPost(frame)!= true)
					Debug::writeln("Echec envoi d'un message");
				// msg sera détruit après le switch
				break;
			}
			case MESSAGE_GET_GPIO_ANS:
				if (buildFrameAndSend(msg)!=true)
					Debug::writeln("Echec envoi d'une frame");
				// Attention : un buffer est alloué dans buildFrameAndSend. Comment le libérer proprement ?
				break;
			default:
				Debug::writeln("Message inconnu");
				break;
			}

			delete(msg);
		}
	}
}

// Méthode appelée par la tache receiveCommandTask_
void App::commandsManagment(void) {
	RawFrameMessage* msg=nullptr;
	uint8_t headerBuffer[3];
	uint8_t* dataFieldBuffer=nullptr;

	Debug::writeln("[App] Demarrage de la tache commandsManagment");

	/*
	 * Reception des trames venant de la raspberry
	 * Les trames sont de la forme:
	 * <SOF><Length><Type><Pdu><Checksum>
	 * | Header     |    Data           |
	 *
	 * SOF = 0x7F
	 * Length est codé sur 2 octets (max 1536) et indique la longueur de la zone data
	 * Type est sur 1 octet et indique le type de frame
	 * Pdu est de longueur variable
	 * Checksum est sur 1 octet et correspond à la somme de calcul de l'ensemble de la
	 * frame, checksum exclu. L'ajout du checksum dans la somme doit donner 0
	 */
	while (1) {
		// Reception du champs "Header"
		if (comRaspberry_->read(headerBuffer, sizeof(headerBuffer)) == HAL_OK ) {
			Debug::writeln("[App] Reception de donnee");

			// Vérification du SOF
			if (headerBuffer[0] != SOF) {
				continue; // Trame invalide, on retourne au début du while et on attend la suivante
			}

			// Lecture de la longueur de la trame
			uint16_t dataFieldLength = (headerBuffer[1]<<8) + headerBuffer[2];

			// La zone data fait au moins 2 octets (type + checksum)
			// et ne peut pas être plus grande que 1536 octets (1.5 * 1024)
			// Si la longueur est invalide, on retourne au début du while et on attend la suivante
			if ((dataFieldLength <2) || (dataFieldLength >1536))
				continue;

			// Allocation dynamique pour le reste de la trame
			dataFieldBuffer = static_cast<uint8_t*>(malloc(dataFieldLength));

			if (dataFieldBuffer == nullptr) {
				// Gestion de l'erreur d'allocation
				continue;
			}

			// Réception du reste de la trame (zone data)
			if (comRaspberry_->read(dataFieldBuffer, dataFieldLength) == HAL_OK ) {

				// Calcul et vérification du checksum
				uint8_t calculatedChecksum = headerBuffer[0]+headerBuffer[1]+headerBuffer[2];

				for (int i = 0; i < dataFieldLength; i++) {
					calculatedChecksum += dataFieldBuffer[i];
				}

				// Le checksum doit être égal à 0
				if (calculatedChecksum != 0) {
					free(dataFieldBuffer); // Libérer la mémoire si la trame est invalide
					dataFieldBuffer = NULL;
					continue;
				}

				/*
				 * La trame semble correcte.
				 * On l'encapsule dans un message et on la poste dans la mailbox
				 * de l'application pour qu'elle soit analysée et envoyée au bon service
				 */

				msg = new RawFrameMessage(dataFieldBuffer, dataFieldLength);
				messageQueue_.send(msg);

				//				comRaspberry_->write(cmdHeader_, sizeof(cmdHeader_));
				//				msg = new GpioMessage({cmdHeader_[0], cmdHeader_[1]});
				//
				//				//queue = gpio_->getQueueHandle();
				//				//if (queue != nullptr) {
				//				if (gpio_->postMessage(msg))
				//					//if (xQueueSend(queue, (void*)&msg, pdMS_TO_TICKS(100)) == pdPASS)
				//					Debug::writeln("[App] Envoi du message OK");
				//				else
				//					Debug::writeln("[App] Echec envoi du message");
				//				//gpio_->postMessage(msg);
				//				//} else
				//				//	Debug::writeln("[App] Queue invalide");
				//			} else {
				//				Debug::writeln("[App] Error when receiving uart data");
			}
		}
	}
}
