/*
 * messages.h
 *
 *  Created on: May 20, 2025
 *      Author: dimercur
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#include "FreeRTOS.h"
#include "queue.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
	MSG_ID_EMPTY					=0,
	MSG_ID_PROBE_REQUEST,
	MSG_ID_PROBE_ANSWER,
	MSG_ID_I2C_SENSORS_10MS_EVENT,
	MSG_ID_I2C_SENSORS_50MS_EVENT
} Messages_ID_TypeDef;

typedef struct {
	Messages_ID_TypeDef id;
	QueueHandle_t *sender;
	uint8_t *data;
	uint16_t length;
} Messages_TypeDef;

/**
 * @brief  Crée un nouveau message
 * @param  id_type: ID du message
 * @param  sender_ptr: Pointeur vers l'expéditeur du message
 * @retval Pointeur vers le message créé
 */
#define NEW_MESSAGE(id_type, sender_ptr)                           \
    ({                                                             \
        Messages_TypeDef *msg = (Messages_TypeDef *)malloc(sizeof(Messages_TypeDef)); \
        if (msg) {                                                 \
            msg->id = (id_type);                                   \
            msg->sender = (sender_ptr);                            \
            msg->data = NULL;                                      \
            msg->length = 0;                                       \
        } else {                                                   \
        	printf ("[Messages] Error allocating memory for new message");\
        	assert_param(0);                                       \
        }                                                          \
        msg;                                                       \
    })

/**
 * @brief  Libère la mémoire allouée pour un message
 * @param  msg_ptr: Pointeur vers le message à libérer
 * @retval None
 */
/* le do while est utilisé pour éviter les erreurs de syntaxe
 * (notamment si la macro est utilisée dans une condition)
 */
#define DELETE_MESSAGE(msg_ptr)         \
    do {                                \
        if (msg_ptr) {                  \
            if ((msg_ptr)->data) {      \
                free((msg_ptr)->data);  \
            }                           \
            free(msg_ptr);              \
            (msg_ptr) = NULL;           \
        }                               \
    } while (0)

#endif /* MESSAGES_H_ */
