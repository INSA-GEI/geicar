/*
 * messagehandler.h
 *
 *  Created on: Jan 15, 2025
 *      Author: dimercur
 */

#ifndef MESSAGEHANDLER_H_
#define MESSAGEHANDLER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "messages.h"

#include "iostream"

class MessageHandler {
public:
	MessageHandler() {}
	MessageHandler(const char* queueName);
	virtual ~MessageHandler();

	QueueHandle_t getQueueHandler();
	std::string toString();

	bool create (const char* queueName);
	bool create (const char* queueName, uint32_t queueSize, uint32_t itemSize);

	// Get latest message from messagequeue, waiting given timeout (by default infinite)
	// If message ptr is null, then a timeout has occured
	Message* get(TickType_t timeout=portMAX_DELAY);
	Message* getFromISR(void);

	// Send message
	bool send(Message* msg, TickType_t timeout=portMAX_DELAY);
	bool sendFromISR(Message* msg);
private:
	const char* messageQueueName_= nullptr;  // Nom de la mailbox
	QueueHandle_t messageQueue_=nullptr; // Handle de la mailbox
};

#endif /* MESSAGEHANDLER_H_ */
