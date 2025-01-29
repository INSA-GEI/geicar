/*
 * taskhandler.h
 *
 *  Created on: Jan 10, 2025
 *      Author: dimercur
 */

#ifndef TASKHANDLER_H_
#define TASKHANDLER_H_

#include <iostream>
#include <functional>
#include <memory>

#include "FreeRTOS.h"
#include "task.h"

class TaskHandler {
public:
	TaskHandler();
	virtual ~TaskHandler();

	bool create(
			std::function<void()> taskFunction,
			const char *taskName,
			configSTACK_DEPTH_TYPE stackSize,
			UBaseType_t prio);  // Méthode pour creer la tâche
	void run(void);     // Méthode pour lancer la tâche
	void suspend(void); // Méthode pour mettre la tâche en suspend
	TaskHandle_t* getTaskHandler(void);

	std::string toString(void);

protected:
	const char* taskName_=nullptr;  // Nom de la tâche
	TaskHandle_t taskHandle_=nullptr;  // Handle de la tâche

	void* parentObject_=nullptr;
	std::function<void()> taskFunction_;

	// Le point d'entrée réél de la tâche, utilisé par FreeRTOS
	static void taskEntry_(void* parameter);
};

#endif /* TASKHANDLER_H_ */
