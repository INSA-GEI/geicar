/*
 * App.h
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#ifndef APP_H_
#define APP_H_

#include <Devices/gpio.h>
#include "stm32u5xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "taskhandler.h"
#include "messagehandler.h"

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
	Gpio *gpio;

	UART_HandleTypeDef huart_;
	SemaphoreHandle_t txCompleteSemaphore_;
	SemaphoreHandle_t rxCompleteSemaphore_;

	uint8_t cmdHeader[3];
	static void txCompleteCallback_(UART_HandleTypeDef* huart);
	static void rxCompleteCallback_(UART_HandleTypeDef* huart);
	void onTxCompleteCallback(void);
	void onRxCompleteCallback(void);

	// Méthode de la classe appelée par la tâche mainTask_
	void mainTask(void);

	// Méthode appelée par la tache receiveCommandTask_
	void receiveFrameTask(void);

};

#endif /* APP_H_ */
