#include "message.h"



#define QUEUE_SIZE 16

QueueHandle_t IMU_Mailbox;
QueueHandle_t GPS_Mailbox;

void  MESSAGE_Init(void) {
	IMU_Mailbox = xQueueCreate(QUEUE_SIZE, sizeof(MESSAGE_Typedef));
	GPS_Mailbox = xQueueCreate(QUEUE_SIZE, sizeof(MESSAGE_Typedef));


	/* Add queues to registry in order to view them in stm32cube ide */
	vQueueAddToRegistry(IMU_Mailbox,"IMU Mailbox");
	vQueueAddToRegistry(GPS_Mailbox,"GPS Mailbox");
}


MESSAGE_Typedef MESSAGE_ReadMailbox(QueueHandle_t mbx) {
	MESSAGE_Typedef msg= {0};
	char msg_received =0;

	while (!msg_received) {
		if (xQueueReceive(mbx, &msg, portMAX_DELAY)) { // un message à été reçu
			msg_received=1;

			return msg;
		}
	}

	return msg;
}

/**
 * @brief  Return first message a given mailbox (non blocking)
 *
 * @param[in] mbx Mailbox reference
 * @return First message in mailbox
 *
 * @remark This function is non blocking.
 * If mailbox is empty when calling the function, a message structure is still returned with \ref MSG_ID_NO_MESSAGE
 * in message id field
 */
MESSAGE_Typedef MESSAGE_ReadMailboxNoDelay(QueueHandle_t mbx) {
	MESSAGE_Typedef msg= {0};

	if (!xQueueReceive(mbx, &msg, 0))  // Pas de message dans la mailbox
		msg.id=MSG_ID_NO_MESSAGE;

	return msg;
}

/**
 * @brief  Post a message in a mailbox with given id and data
 *
 * @param[in] mbx_dest Destination mailbox
 * @param[in] id id of message (as found in \ref msg_id_def "Message ID definitions")
 * @param[in] mbx_sender Sender mailbox (may be null if no answer is expected)
 * @param[in] data Reference to data to store in message structure
 * @return None
 *
 * @remark This function is non blocking.
 * If mailbox is full when calling the function, error will be ignored silently
 */
void MESSAGE_SendMailbox(QueueHandle_t mbx_dest, uint16_t id, QueueHandle_t mbx_sender, void *data){
	MESSAGE_Typedef msg;

	msg.id=id;
	msg.sender = &mbx_sender;
	msg.data=data;

	if (!xQueueSend(mbx_dest, &msg, 0)) { // envoi sans attendre
		printf("Failed to send data, Queue full.\r\n");
	}
}

/**
 * @brief  Post, from an ISR, a message in a mailbox with given id and data
 *
 * @param[in] mbx_dest Destination mailbox
 * @param[in] id id of message (as found in \ref msg_id_def "Message ID definitions")
 * @param[in] mbx_sender Sender mailbox (may be null if no answer is expected)
 * @param[in] data Reference to data to store in message structure
 * @param[in] xHigherPriorityTaskWoken Reference to flag indicating if a context switch is to be done at end of ISR
 * @return None
 *
 * @remark This function is non blocking.
 * If mailbox is full when calling the function, error will be ignored silently
 */
void MESSAGE_SendMailboxFromISR(QueueHandle_t mbx_dest, uint16_t id, QueueHandle_t mbx_sender, void *data, BaseType_t *xHigherPriorityTaskWoken) {
	MESSAGE_Typedef msg;

	msg.id=id;
	msg.sender = &mbx_sender;
	msg.data=data;

	if (!xQueueSendFromISR( mbx_dest, &msg, xHigherPriorityTaskWoken)) { // envoi sans attendre
		//printf("Failed to send data, Queue full.\r\n");
	}
}
