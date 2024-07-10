#ifndef MESSAGE_H
#define MESSAGE_H

#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
	uint16_t id;			/**< Id of message, as found in \ref msg_id_def "Message ID definitions" */
	QueueHandle_t* sender;	/**< reference to sender mailbox, in case of expected answer */
	void *data;				/**< reference to data */
} MESSAGE_Typedef;

extern QueueHandle_t IMU_Mailbox;
extern QueueHandle_t GPS_Mailbox;
extern QueueHandle_t Appli_Mailbox;

#define MSG_ID_NO_MESSAGE			0x00
#define MSG_ID_IMU_TEMP					0x20
#define MSG_ID_IMU_HUM					0x21
#define MSG_ID_IMU_GYR					0x22
#define MSG_ID_IMU_MAG					0x23
#define MSG_ID_IMU_ACC					0x24
#define MSG_ID_IMU_PRESS				0x25


#define MSG_ID_GPS					0x30

void MESSAGE_Init(void);
MESSAGE_Typedef MESSAGE_ReadMailbox(QueueHandle_t mbx);
MESSAGE_Typedef MESSAGE_ReadMailboxNoDelay(QueueHandle_t mbx);
void MESSAGE_SendMailbox(QueueHandle_t mbx_dest, uint16_t id, QueueHandle_t mbx_sender, void *data);
void MESSAGE_SendMailboxFromISR(QueueHandle_t mbx_dest, uint16_t id, QueueHandle_t mbx_sender, void *data, BaseType_t *xHigherPriorityTaskWoken);

#endif
