/*
 * App.h
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#ifndef APP_H_
#define APP_H_

#include "stm32u5xx_hal.h"

#include "FreeRTOS.h"

#include "taskhandler.h"
#include "messagehandler.h"

#include "debug.h"

#include <Devices/gpio.h>
#include <Drivers/uartdriver.h>

class App {
public:
	App();
	~App();

	void run(void);

private:
	MessageHandler messageQueue_; // Objet mailbox
	TaskHandler mailboxManagmentTask_;
	TaskHandler commandsManagmentTask_;

	// uart for communication
	UartDriver *comRaspberry_;

	// Pour les rapports périodiques
	Debug *debug;

	// Périphériques
	Gpio *gpio_;

	void probe(void);

	bool parseFrameAndPost(RawFrameMessage *frame);
	bool buildFrameAndSend(Message *msg);
	uint8_t computeChecksum(uint8_t* buffer, uint16_t length);

	void onTxCompleteCallback(void);
	void onRxCompleteCallback(void);

	// Méthode de la classe appelée par la tâche mailboxManagmentTask_
	void mailboxManagment(void);

	// Méthode appelée par la tache commandsManagmentTask_
	void commandsManagment(void);
};

#endif /* APP_H_ */
