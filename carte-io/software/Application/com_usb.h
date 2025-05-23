/*
 * com-usb.h
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#ifndef COM_USB_H_
#define COM_USB_H_

#include "messages.h"

/**
 * @brief: Initialisation de la communication USB
 * @param AppMsgQueue: Pointeur vers la file de messages de l'application
 * @retval None
 */
void COM_USB_Init(QueueHandle_t *AppMsgQueue);

/**
 * @brief  Envoie une trame de données sur le port USB
 * @param  msg: Pointeur vers le message à envoyer
 * @retval HAL_StatusTypeDef: Statut de l'envoi
 */
HAL_StatusTypeDef COM_USB_SendData(Messages_TypeDef *msg);

#endif /* COM_USB_H_ */
