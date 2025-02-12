/*
 * sensor.h
 *
 *  Created on: Feb 10, 2025
 *      Author: dimercur
 */

#ifndef DEVICES_SENSOR_H_
#define DEVICES_SENSOR_H_

#include "messagehandler.h"
#include "messages.h"

class Sensor {
public:
	Sensor() {}
	Sensor(const char *mailboxName, MessageHandler &app_mailbox);

	virtual ~Sensor() {}

	virtual void setAppMailbox(MessageHandler &app_mailbox) {appMailbox_ = &app_mailbox;}

	virtual bool post(Message &msg);
	virtual bool postFromISR(Message &msg);

	static bool probe(void* handler) {return false;}
protected:
	MessageHandler mailbox_; // Objet mailbox
	MessageHandler* appMailbox_ = nullptr;
};

#endif /* DEVICES_SENSOR_H_ */
