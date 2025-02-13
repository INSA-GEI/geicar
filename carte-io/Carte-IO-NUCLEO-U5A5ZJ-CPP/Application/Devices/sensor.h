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

	virtual bool postMessage(Message &msg);
	virtual bool postMessageFromISR(Message &msg);
	virtual bool postMessage(Message* msg);
	virtual bool postMessageFromISR(Message* msg);

	static bool probe(void* handler) {return false;}
protected:
	MessageHandler mailbox_; // Objet mailbox
	MessageHandler* appMailbox_ = nullptr;

	virtual bool postMessageToApp(Message &msg);
	virtual bool postMessageToAppFromISR(Message &msg);
	virtual bool postMessageToApp(Message* msg);
	virtual bool postMessageToAppFromISR(Message* msg);
};

#endif /* DEVICES_SENSOR_H_ */
