/*
 * gpio.c
 *
 *  Created on: May 22, 2025
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"
#include "gpio.h"

#include "config.h"
#include "messages.h"

#include "FreeRTOS.h"
#include "timers.h"

#include <stdio.h>
#include <stdlib.h>

#include "main.h" // Pour les définitions de GPIO_Pin_XX

uint32_t GPIO_Mode[4] = {GPIO_MODE_INPUT}; // 4 GPIOs max

static QueueHandle_t *ApplicationMessageQueue; // Handle de la file de messages de l'application

/**
 * @brief  Fonction d'initialisation des GPIO
 * @param  AppMsgQueue: Pointeur vers la file de messages de l'application
 * @retval None
 */
void GPIO_Init(QueueHandle_t *AppMsgQueue) {
	assert_param(AppMsgQueue != NULL);
	ApplicationMessageQueue = AppMsgQueue;

	printf("[GPIO] Initialisation... ");

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE(); // pour LED_ACTIVITY_Pin et PWR_CSLEEP_Pin
	__HAL_RCC_GPIOE_CLK_ENABLE(); // pour POWEROFF_Pin et IOx_Pin

	/*Configure GPIO pins : LED_ACTIVITY_Pin PWR_CSLEEP_Pin */
	GPIO_InitStruct.Pin = LED_ACTIVITY_Pin | PWR_CSLEEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_ACTIVITY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : POWEROFF_Pin */
	// Configuration à l'etat haut, avant de basculer la ligne en sortie
	// Ceci evite le transitoire à l'état bas de la ligne

	HAL_GPIO_WritePin(POWEROFF_GPIO_Port, POWEROFF_Pin, GPIO_PIN_SET);
	// Configuration de la ligne en sortie
	GPIO_InitStruct.Pin = POWEROFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(POWEROFF_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : IO1_Pin IO2_Pin IO3_Pin IO4_Pin */
	GPIO_InitStruct.Pin = IO1_Pin | IO2_Pin | IO3_Pin | IO4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IO1_GPIO_Port, &GPIO_InitStruct);

	printf("[GPIO] Done\n");
}

/**
 * @brief  Fonction de décodage du parametre param1 d'un message
 * @param  param: Paramètre d'entrée
 * @retval uint32_t: GPIO pins decodées
 */
uint32_t GPIO_DecodePin(uint32_t param) {
	uint32_t pins = 0;

	if (param & 0x1) pins|=IO1_Pin;
	if (param & 0x2) pins|=IO2_Pin;
	if (param & 0x4) pins|=IO3_Pin;
	if (param & 0x8) pins|=IO4_Pin;

	return pins;
}

/**
 * @brief  Fonction de décodage du parametre param2 d'un message
 * @param  param: Paramètre d'entrée
 * @retval uint16_t: pins mode decodées
 */
uint16_t GPIO_DecodeMode(uint32_t param) {
	uint16_t mode = 0;

	if (param == 0x0)
		mode = GPIO_MODE_INPUT;
	else if (param == 0x1)
		mode= GPIO_MODE_OUTPUT_PP;
	else
		mode = GPIO_MODE_INPUT;

	return mode;
}
/**
 * @brief  Fonction de traitement des messages pour les GPIO
 * @param  msg: Pointeur vers le message à traiter
 * @retval HAL_StatusTypeDef: Statut de la fonction
 */
HAL_StatusTypeDef GPIO_MessageProcessor(Messages_TypeDef *msg) {
	assert_param(msg != NULL);
	assert_param(ApplicationMessageQueue != NULL);

	/* la structure Messages_TypeDef contient 2 parametres:
	 * param1: Pins GPIO (GPIO_PIN_XX): bit0 = 1 => IO_1, bit1 = 1 => IO_2, etc.
	 * param2: Mode GPIO (GPIO_MODE_XX): 0= GPIO_MODE_INPUT, 1=GPIO_MODE_OUTPUT_PP, etc.
	 */
	if (msg->id == MSG_ID_GPIO_CONFIGURE) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		GPIO_InitStruct.Pin = GPIO_DecodePin(msg->param1);
		GPIO_InitStruct.Mode = GPIO_DecodeMode(msg->param2);


		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		printf("[GPIO] GPIO %ld configuré en mode %ld\n", GPIO_InitStruct.Pin,
				GPIO_InitStruct.Mode);

	} else if (msg->id == MSG_ID_GPIO_SET_STATE) {
		if (msg->param1 & 0x1) HAL_GPIO_WritePin(GPIOE, IO1_Pin, msg->param2 & 0x01);
		if (msg->param1 & 0x2) HAL_GPIO_WritePin(GPIOE, IO2_Pin, msg->param2 & 0x02);
		if (msg->param1 & 0x4) HAL_GPIO_WritePin(GPIOE, IO3_Pin, msg->param2 & 0x04);
		if (msg->param1 & 0x8) HAL_GPIO_WritePin(GPIOE, IO4_Pin, msg->param2 & 0x08);

		printf("[GPIO] GPIO %ld mis à l'état %ld\n", msg->param1, msg->param2);
	} else if (msg->id == MSG_ID_GPIO_GET_STATE) {
		uint32_t pins = GPIO_DecodePin(msg->param1);
		GPIO_PinState state = HAL_GPIO_ReadPin(GPIOE, pins);
		printf("[GPIO] GPIO %ld est à l'état %d\n", msg->param1, state);
	} else {
		printf("[GPIO] Message non traité : ID=%d\n", msg->id);
	}

	return HAL_OK;
}
