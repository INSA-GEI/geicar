/*
 * sensor.cpp
 *
 *  Created on: Feb 12, 2025
 *      Author: dimercur
 */

#include "sensor.h"
#include "debug.h"

/**
 * @brief Initialise la mailbox du capteur
 *
 * Permet d'initialiser la mailbox du capteur.
 * @param queueName Non de la mailbox affichée par Freertos
 * @return true si la creation reussi, false sinon
 */
bool Sensor::initMessagesManagement(const char *queueName) {
	if (!mailbox_.create(queueName)) {
		PANIC ("[Sensor] Impossible de creer la mailbox");
		return false;
	} else
		return true;
}

bool Sensor::postMessage(Message &msg) {
	bool status = false;

	/* Envoi d'un message vers le capteur */
	status = mailbox_.send(&msg);

	return status;
}

bool Sensor::postMessageFromISR(Message &msg) {
	bool status = false;

	status = mailbox_.sendFromISR(&msg);

	return status;
}

bool Sensor::postMessage(Message* msg) {
	bool status = false;

	/* Envoi d'un message vers le capteur */
	if (msg)
		status = mailbox_.send(msg);

	return status;
}

bool Sensor::postMessageFromISR(Message* msg) {
	bool status = false;

	if (msg)
		status = mailbox_.sendFromISR(msg);

	return status;
}

bool Sensor::postMessageToApp(Message &msg) {
	bool status = false;

	/* Envoi d'un message vers le capteur */
	status = appMailbox_->send(&msg);

	return status;
}

bool Sensor::postMessageToAppFromISR(Message &msg) {
	bool status = false;

	status = appMailbox_->sendFromISR(&msg);

	return status;
}

bool Sensor::postMessageToApp(Message* msg) {
	bool status = false;

	/* Envoi d'un message vers le capteur */
	if (msg)
		status = appMailbox_->send(msg);

	return status;
}

bool Sensor::postMessageToAppFromISR(Message* msg) {
	bool status = false;

	if (msg)
		status = appMailbox_->sendFromISR(msg);

	return status;
}

