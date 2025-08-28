/*
 * configuration.h
 *
 *  Created on: Aug 27, 2025
 *      Author: dimercur
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include "main.h"

// Application version
#define APP_VERSION "1.0.0"

//L/R buttons
#define RightSteeringButton_Pin 		GPIO_PIN_14
#define RightSteeringButton_GPIO_Port 	GPIOB
#define LeftSteeringButton_Pin 			GPIO_PIN_15
#define LeftSteeringButton_GPIO_Port 	GPIOB

//US
#define US_Front_Left_Trig_Pin 	GPIO_PIN_1
#define US_Front_Center_Trig_Pin GPIO_PIN_2
#define US_Front_Right_Trig_Pin GPIO_PIN_3

#define US_Rear_Left_Trig_Pin 	GPIO_PIN_4
#define US_Rear_Center_Trig_Pin GPIO_PIN_5
#define US_Rear_Right_Trig_Pin 	GPIO_PIN_7

#define US_Right_Echo_Pin 		GPIO_PIN_6
#define US_Left_Echo_Pin 		GPIO_PIN_8
#define US_Center_Echo_Pin 		GPIO_PIN_9

#define US_GPIO_Port 			GPIOC

// Periodic events, in ms
#define PERIOD_CAR_CONTROL_LOOP 30 		// Period in ms for car control loop
										// (30 ms is, because, at max speed, speed sensors update at 25-30 ms
                                        // No need to be faster
#define PERIOD_UPDATE_US 		50 		// Period in ms to update us data
#define PERIOD_SEND_MOTORS 		100		// Period in ms to send motors data (speed and odometers)
#define PERIOD_SEND_BATT 		2000 	// Period in ms to send battery level
#define DEBUG_LOOP_PERIOD_MS	1000	// Debug loop period in ms

#define TIM_MOTOR_LEFT 			TIM1
#define TIM_MOTOR_RIGHT 		TIM1
#define TIM_STEERING 			TIM4
#define TIM_ENCODER_LEFT 		TIM2
#define TIM_ENCODER_RIGHT 		TIM2
#define TIM_US 					TIM3
#define TIM_MOTOR_LEFT_CHANNEL 	TIM_CHANNEL_1
#define TIM_MOTOR_RIGHT_CHANNEL TIM_CHANNEL_2
#define TIM_STEERING_CHANNEL 	TIM_CHANNEL_1
#define TIM_ENCODER_LEFT_CHANNEL 	TIM_CHANNEL_3
#define TIM_ENCODER_RIGHT_CHANNEL 	TIM_CHANNEL_1

// Tasks, semaphores, queues constants
#define APPLOOP_QUEUE_LENGTH   	10
#define APPLOOP_QUEUE_ITEM_SIZE sizeof(void*)   // Chaque élément est un pointeur

// Tasks stack sizes
#define APPLOOP_TASK_STACK_SIZE 	256   	// en mots de 32 bits
#define DEBUGLOOP_TASK_STACK_SIZE 	256 	// en mots de 32 bits. Besoin de pas mal d'espace pour la fonction sprintf
#define USLOOP_TASK_STACK_SIZE  	128 	// en mots de 32 bits. Pas besoin d'une stack enorme
#define CONTROLLOOP_TASK_STACK_SIZE 256 	// en mots de 32 bits. A revoir plus tard mais dans le doute, si on fait pas mal de calcul, prevoir de la place.
#define CALIBRATION_TASK_STACK_SIZE 128 	// en mots de 32 bits. Pas besoin d'une stack enorme

// Tasks priorities
#define USLOOP_TASK_PRIORITY    	(tskIDLE_PRIORITY + 5) // Highest priority, no blocking function inside
#define CONTROLLOOP_TASK_PRIORITY 	(tskIDLE_PRIORITY + 4)
#define APPLOOP_TASK_PRIORITY   	(tskIDLE_PRIORITY + 3)
#define CALIBRATION_TASK_PRIORITY 	(tskIDLE_PRIORITY + 2)
#define DEBUGLOOP_TASK_PRIORITY 	(tskIDLE_PRIORITY + 1) // Lowest priority

// APP messages IDs
//#define CAR_CONTROL_UPDATE_ID 	1
#define US_UPDATE_ID 			2
#define CAN_SEND_MOTORS_ID		3
#define CAN_SEND_US_ID			4
#define CAN_SEND_BATT_ID		5
//#define COMM_CHECKING_ID		6
//#define START_CALIBRATION_ID	7
#define CAN_RECEIVED_FRAME_ID	8

#endif /* CONFIGURATION_H_ */
