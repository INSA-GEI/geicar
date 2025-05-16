/*
 * config.h
 *
 *  Created on: Dec 20, 2024
 *      Author: dimercur
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Version
#define MAJOR_VER	1
#define MINOR_VER	0
#define VERSION_STRING "1.0"

// Tasks priority, higher = higher priority
#define TASK_PRIO_APP_RCV_CMD		20
#define TASK_PRIO_APP_MSG_HANDLER	19
#define TASK_PRIO_PROBE_RUN			18
#define TASK_PRIO_PERIODIC_DEBUG	1

// Various config
#define TASK_STACK_SIZE_STD				256
#define TASK_STACK_SIZE_APPLICATION		2*256

// UARTs configuration
#define APP_UART_CIRCULAR_BUFFER_SIZE	20

#endif /* CONFIG_H_ */
