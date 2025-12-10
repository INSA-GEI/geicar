/*
 * app_config.h
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

// Version
#define MAJOR_VER						1
#define MINOR_VER						0
#define VERSION_STRING 					"1.0"

// Tasks priority, higher = higher priority
#define TASK_PRIO_APP_CMD_MGMT_CMD		tskIDLE_PRIORITY +20
#define TASK_PRIO_APP_MBX_MGMT_TASK		tskIDLE_PRIORITY +19
#define TASK_PRIO_PROBE_RUN				tskIDLE_PRIORITY +18
#define TASK_PRIO_GPIO					tskIDLE_PRIORITY +17

#define TASK_PRIO_PERIODIC_DEBUG		tskIDLE_PRIORITY + 1

// Stack size config
#ifdef DEBUG
#define TASK_STACK_SIZE_STD				96		// Attention, c'est exprimé en nombre de mots 32 bits,
												// il faut donc multiplier cette valeur par 4 pour avoir la taille
                                                // réelle de la stack qui sera allouée
#define TASK_STACK_SIZE_APPLICATION		3*96	// Voir la remarque ci dessus
#define TASK_STACK_SIZE_DEBUG			96		// Voir la remarque ci dessus
#else
#define TASK_STACK_SIZE_STD				96		// Attention, c'est exprimé en nombre de mots 32 bits,
												// il faut donc multiplier cette valeur par 4 pour avoir la taille
                                                // réelle de la stack qui sera allouée
#define TASK_STACK_SIZE_APPLICATION		3*96	// Voir la remarque ci dessus
#define TASK_STACK_SIZE_DEBUG			16		// Voir la remarque ci dessus
#endif /* DEBUG */

// Queues configuration
#define QUEUE_LENGTH 					6
#define ITEM_SIZE 						sizeof(void*)

// UART VCP
#define UART_VCP	UART1

#endif /* APP_CONFIG_H_ */
