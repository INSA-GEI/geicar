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

#include <Devices/gpio.h>
#include <Drivers/uartdriver.h>

class App {
public:
	App(const char* taskName, const char* queueName);
	App() {App("Application", "App_Queue");}

	void run(void);
	virtual ~App();

private:
	MessageHandler messageQueue_; // Objet mailbox
	TaskHandler mainTask_;
	TaskHandler receiveCommandTask_;

	// Peripheriques
	Gpio *gpio_;

	// uart data for communication
	UartDriver *uartDriver_;
//	UART_HandleTypeDef huart_;

	uint8_t cmdHeader_[3];
//	static void txCompleteCallback_(UART_HandleTypeDef* huart);
//	static void rxCompleteCallback_(UART_HandleTypeDef* huart);
	void onTxCompleteCallback(void);
	void onRxCompleteCallback(void);

	// Méthode de la classe appelée par la tâche mainTask_
	void mainTask(void);

	// Méthode appelée par la tache receiveCommandTask_
	void receiveFrameTask(void);
};

#endif /* APP_H_ */
