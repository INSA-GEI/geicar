/*
 * taskhandler.cpp
 *
 *  Created on: Jan 10, 2025
 *      Author: dimercur
 */

#include <taskhandler.h>

TaskHandler::TaskHandler() {
	// TODO Auto-generated constructor stub

}

TaskHandler::~TaskHandler() {
	if (taskHandle_ != nullptr)
		vTaskDelete(taskHandle_);
}

bool TaskHandler::create(
		std::function<void()> taskFunction,
		const char *taskName,
		configSTACK_DEPTH_TYPE stackSize,
		UBaseType_t prio) {
	// Méthode pour creer la tâche

	taskName_ = taskName;
	//parentObject_ = parentObj;
	taskFunction_ = taskFunction;

	// Création de la tâche FreeRTOS, la fonction statique est utilisée comme point d'entrée
	if (xTaskCreate(taskEntry_, taskName_, stackSize, (void*) this, prio, &taskHandle_)!=pdPASS)
		return false;

	/* La tache est soit dans l'etat Pret si l'OS n'est pas lancé, soit l'etat suspendu si l'OS à demarré */
	return true;
}

void TaskHandler::run(void) {
	// Méthode pure virtuelle pour lancer la tâche
	if (taskHandle_ != nullptr) {
		vTaskResume(taskHandle_);
	}
}

void TaskHandler::suspend(void) {
	// Méthode pure virtuelle pour mettre la tâche en suspend
	if (taskHandle_ != nullptr) {
		vTaskSuspend(taskHandle_);
	}
}

TaskHandle_t* TaskHandler::getTaskHandler(void) {
	return &taskHandle_;
}

std::string TaskHandler::toString(void) {
	std::string s = std::string("");

	if (taskName_ != nullptr)
		s="TaskHandler (" + std::string(taskName_)+ ")";

	return s;
}

void TaskHandler::taskEntry_(void* parameter) {
	TaskHandler* instance = static_cast<TaskHandler*>(parameter);
	if (instance) {
		instance->taskFunction_();
	}

	vTaskDelete(nullptr);  // Supprime la tâche si jamais la méthode taskFunction() retourne
}

