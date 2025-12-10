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
	typedef struct {
		uint16_t length;
		uint8_t* data;
	} RawData_Typedef;

	Sensor() = default;
	virtual ~Sensor() {}

	virtual bool initMessagesManagement(const char* queueName);
	virtual bool initMessagesManagement(const char* queueName, MessageHandler &app_mailbox) {
		setApplicationMailbox(app_mailbox);
		return initMessagesManagement(queueName);
	}

	virtual void setApplicationMailbox(MessageHandler &app_mailbox) {appMailbox_ = &app_mailbox;}

	virtual bool postMessage(Message &msg);
	virtual bool postMessageFromISR(Message &msg);
	virtual bool postMessage(Message* msg);
	virtual bool postMessageFromISR(Message* msg);

	static bool probe(void* handler) {return false;}
	static Message* fromFrame(Sensor::RawData_Typedef raw) { return new Message();}
	static Sensor::RawData_Typedef toFrame(Message* msg) {return {0, nullptr};}

protected:
	MessageHandler mailbox_; // Objet mailbox
	MessageHandler* appMailbox_ = nullptr;

	virtual bool postMessageToApp(Message &msg);
	virtual bool postMessageToAppFromISR(Message &msg);
	virtual bool postMessageToApp(Message* msg);
	virtual bool postMessageToAppFromISR(Message* msg);
};

#endif /* DEVICES_SENSOR_H_ */
