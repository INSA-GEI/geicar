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

bool Sensor::post(Message &msg) {
	bool status = false;

	if (appMailbox_ != nullptr)
		status = appMailbox_->send(&msg);

	return status;
}

bool Sensor::postFromISR(Message &msg) {
	bool status = false;

	if (appMailbox_ != nullptr)
		status = appMailbox_->sendFromISR(&msg);

	return status;
}



