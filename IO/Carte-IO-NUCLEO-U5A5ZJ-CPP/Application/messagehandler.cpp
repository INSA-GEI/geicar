/*
 * messagehandler.cpp
 *
 *  Created on: Jan 15, 2025
 *      Author: dimercur
 */

#include <messagehandler.h>
#include "app_config.h"
#include "debug.h"

MessageHandler::MessageHandler(const char* queueName) : messageQueueName_(queueName) {
	if (!create(queueName)) // Si la creation de la queue echoue -> while (1)
		PANIC("[MessageHandler] Impossible de creer la mailbox");
}

MessageHandler::~MessageHandler() {
	// Suppression de la message queue
	if (messageQueue_ != nullptr)
		vQueueDelete(messageQueue_);
}

bool MessageHandler::create (const char* queueName, uint32_t queueSize, uint32_t itemSize) {
	bool status = true;

	/* Creation de la message queue associée à l'objet */
	messageQueue_ = xQueueCreate((UBaseType_t)queueSize, (UBaseType_t)itemSize);
	if (messageQueue_ == NULL) {
		Debug::writeln("[App] Erreur de creation de la file");
		status =false;
	} else {
		messageQueueName_ = queueName;
		vQueueAddToRegistry(messageQueue_, messageQueueName_ );
	}

	return status;
}

bool MessageHandler::create (const char* queueName) {
	return create(queueName, QUEUE_LENGTH, ITEM_SIZE);
}

QueueHandle_t MessageHandler::getQueueHandler() {
	return messageQueue_;
}

std::string MessageHandler::toString() {
	return "Queue : " + std::string(messageQueueName_);
}

// Get latest message from messagequeue, waiting given timeout (by default infinite)
// If message ptr is null, then a timeout has occured
Message* MessageHandler::get(TickType_t timeout) {
	void *p=nullptr;
	Message* msg = nullptr;

	if (messageQueue_) {
		if (xQueueReceive(messageQueue_, static_cast<void*>(&p), timeout) == pdTRUE) {
			if (p)
				msg = static_cast<Message*>(p);
		}
	} else
		PANIC("[MessageHandler] Mailbox non cree");

	return msg;
}

// Get latest message from messagequeue, waiting given timeout (by default infinite)
// If message ptr is null, then a timeout has occured
Message* MessageHandler::getFromISR(void) {
	void *p=nullptr;
	BaseType_t xTaskWokenByReceive = pdFALSE;
	Message* msg = nullptr;

	if (messageQueue_) {
		if (xQueueReceiveFromISR(messageQueue_, static_cast<void*>(&p), &xTaskWokenByReceive) == pdTRUE) {
			if( xTaskWokenByReceive != pdFALSE ) {
				/* We should switch context so the ISR returns to a different task.
		           NOTE: How this is done depends on the port you are using. Check
		           the documentation and examples for your port. */
				taskYIELD ();
			}

			if (p)
				msg = static_cast<Message*>(p);
		}
	} else
		PANIC("[MessageHandler] Mailbox non cree");

	return msg;
}

// Send message
bool MessageHandler::send(Message* msg, TickType_t timeout) {
	if (messageQueue_)
		return (xQueueSend(messageQueue_, static_cast<void*>(&msg), timeout) == pdPASS) ? true:false;
	else {
		PANIC("[MessageHandler] Mailbox non cree");
		return false;
	}
}

// Send message
bool MessageHandler::sendFromISR(Message* msg) {
	BaseType_t xTaskWokenBySend = pdFALSE;

	if (messageQueue_) {
		if (xQueueSendFromISR(messageQueue_, static_cast<void*>(&msg), &xTaskWokenBySend) == pdPASS) {
			if( xTaskWokenBySend != pdFALSE ) {
				/* We should switch context so the ISR returns to a different task.
				           NOTE: How this is done depends on the port you are using. Check
				           the documentation and examples for your port. */
				taskYIELD ();
			}

			return true;
		} else
			return false;
	} else {
		PANIC("[MessageHandler] Mailbox non cree");
		return false;
	}

}
