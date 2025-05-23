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
#define TASK_PRIO_COM_USB_RCV_CMD							20
#define TASK_PRIO_APP_MSG_HANDLER							19

#define TASK_PRIO_I2C_SENSORS_MESSAGES_HANDLER_TASK        	17
#define TASK_PRIO_I2C_SENSORS_PERIODIC_TASK        			16
#define TASK_PRIO_PROBE_RUN									2
#define TASK_PRIO_PERIODIC_DEBUG							1

// Various config
#define TASK_STACK_SIZE_STD				256
#define TASK_STACK_SIZE_APPLICATION		2*256
#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(void*)

// UARTs configuration
#define APP_UART_CIRCULAR_BUFFER_SIZE	20

// I2C configuration
#define I2C_INTERNAL 	I2C1
#define I2C_EXTERNAL 	I2C2
#define I2C_ARBITRARY 	I2C4

// I2C configuration
#define UART_COM_USB 	USART1
#define UART_ARBITRARY 	LPUART1
#define UART_GPS 		USART3
#define UART_LIDAR_1 	UART4
#define UART_LIDAR_2 	UART5

#endif /* CONFIG_H_ */
