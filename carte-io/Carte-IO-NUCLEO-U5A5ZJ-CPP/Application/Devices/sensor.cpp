/*
 * sensor.cpp
 *
 *  Created on: Feb 12, 2025
 *      Author: dimercur
 */

#include "sensor.h"
#include "debug.h"

Sensor::Sensor(const char *mailboxName, MessageHandler &app_mailbox): appMailbox_(&app_mailbox) {
	if (!mailbox_.create(mailboxName))
		PANIC ("[Sensor] Impossible de créer la mailbox");
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
	status = mailbox_.send(msg);

	return status;
}

bool Sensor::postMessageFromISR(Message* msg) {
	bool status = false;

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
	status = appMailbox_->send(msg);

	return status;
}

bool Sensor::postMessageToAppFromISR(Message* msg) {
	bool status = false;

	status = appMailbox_->sendFromISR(msg);

	return status;
}

