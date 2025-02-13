/*
 * app_config.h
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

// Version
#define MAJOR_VER					1
#define MINOR_VER					0
#define VERSION_STRING 				"1.0"

// Tasks priority, higher = higher priority
#define TASK_PRIO_APP_RCV_CMD		tskIDLE_PRIORITY +20
#define TASK_PRIO_APP_MAIN_TASK		tskIDLE_PRIORITY +19
#define TASK_PRIO_PROBE_RUN			tskIDLE_PRIORITY +18
#define TASK_PRIO_PERIODIC_DEBUG	tskIDLE_PRIORITY + 1

// Various config
#define TASK_STACK_SIZE_STD				256
#define TASK_STACK_SIZE_APPLICATION		4*256

// Queues configuration
#define QUEUE_LENGTH 				5
#define ITEM_SIZE 					sizeof(void*)

// UART VCP
#define UART_VCP	UART1

#endif /* APP_CONFIG_H_ */
