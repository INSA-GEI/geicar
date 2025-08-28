/**
 * @file    wheels.c
 * @author  Sebastien DI MERCURIO
 * @version V1.0
 * @date    20 Aout 2023
 * @brief   Functions to control the propulsion wheels of the car.
 */

#include "wheels.h"
#include "configuration.h"

#include "tim.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "tasks.h" // for xAppLoopQueue"

uint32_t VMG_mes = 0, VMD_mes = 0, per_vitesseG = 0, per_vitesseD = 0;
int nbImpulsionG = 0;
int nbImpulsionD = 0;

// Variables globales pour mémoriser la dernière capture
uint32_t lastCaptureG = 0;
uint32_t lastCaptureD = 0;

// Compteurs d'overflow par moteur
uint8_t ovfCountG = 0;
uint8_t ovfCountD = 0;

void WHEELS_Init() {
	/* PWM MOTEURS */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	//Sorties complementaires
	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_3);

	/*Vitesse*/
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	//__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT (&htim2,TIM_CHANNEL_3);//autorisation IT capture CH3 (moteur gauche)
	HAL_TIM_IC_Start_IT (&htim2,TIM_CHANNEL_1);//autorisation IT capture CH1 (moteur droit)
}

void WHEELS_SetSpeed(GPIO_PinState en_right, GPIO_PinState en_left, int speed_right, int speed_left){

	//Normalize value between MAX_SPEED_FORWARD and MAX_SPEED_BACKWARD (because CAN order is between 0 and 100)
	speed_right = ((MAX_SPEED_FORWARD-MAX_SPEED_BACKWARD)/100.0) * speed_right + MAX_SPEED_BACKWARD;
	speed_left = ((MAX_SPEED_FORWARD-MAX_SPEED_BACKWARD)/100.0) * speed_left + MAX_SPEED_BACKWARD;

	/* Threshold rotating speed of propulsion wheels*/
	if (speed_left > MAX_SPEED_FORWARD) {
		speed_left = MAX_SPEED_FORWARD;
	} else if (speed_left < MAX_SPEED_BACKWARD)
		speed_left = MAX_SPEED_BACKWARD;


	if (speed_right > MAX_SPEED_FORWARD) {
		speed_right = MAX_SPEED_FORWARD;
	} else if (speed_right < MAX_SPEED_BACKWARD)
		speed_right = MAX_SPEED_BACKWARD;

	speed_left = 3200 * ( speed_left/ 100.0 );
	speed_right = 3200 * ( speed_right/ 100.0 );

	TIM1->CCR1=speed_left;
	TIM1->CCR2=speed_right;

	/*        Enable moteurs        */
	/* GPIO_PIN_SET : activation    */
	/* GPIO_PIN_RESET : pont ouvert */

	HAL_GPIO_WritePin( GPIOC, GPIO_PIN_10, en_left); //PC10  Right Rear
	HAL_GPIO_WritePin( GPIOC, GPIO_PIN_11, en_right); //PC11  Left Rear
}

uint32_t WHEELS_GetSensor(uint8_t motor) {
uint32_t value=0;
	if (motor == WHEELS_MOTOR_LEFT) {
		value = VMG_mes; // Left motor
	} else if (motor == WHEELS_MOTOR_RIGHT) {
		value = VMD_mes; // right motor
	}

	return value;
}

uint32_t WHEELS_GetPERVitesse(uint8_t motor) {
uint32_t value=0;
	if (motor == WHEELS_MOTOR_LEFT) {
		value = per_vitesseG; // Left motor
	} else if (motor == WHEELS_MOTOR_RIGHT) {
		value = per_vitesseD; // right motor
	}

	return value;
}

int WHEELS_GetOdometer(uint8_t motor) {
int value=0;
	if (motor == WHEELS_MOTOR_LEFT) {
		value = nbImpulsionG; // Left motor
	} else if (motor == WHEELS_MOTOR_RIGHT) {
		value = nbImpulsionD; // right motor
	}

	return value;
}

void WHEELS_ResetOdometer(uint8_t motor) {
	if (motor == WHEELS_MOTOR_LEFT) {
		nbImpulsionG = 0; // Left motor
	} else if (motor == WHEELS_MOTOR_RIGHT) {
		nbImpulsionD = 0; // right motor
	}
}
void WHEELS_OverflowManager() {
	// Incrémentation des compteurs d’overflow
	if (++ovfCountG >= 2) {
		VMG_mes = 0;     // moteur gauche arrêté
		ovfCountG = 2;   // éviter overflow du compteur logiciel
	}

	if (++ovfCountD >= 2) {
		VMD_mes = 0;     // moteur droit arrêté
		ovfCountD = 2;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	uint32_t capture;
	uint32_t diff;

	if ((htim->Instance == TIM_ENCODER_LEFT) || (htim->Instance == TIM_ENCODER_RIGHT))	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {   // Moteur droit
			capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);


			if (capture >= lastCaptureD)
				diff = capture - lastCaptureD;
			else
				diff = (0xFFFF - lastCaptureD) + capture + 1;

			lastCaptureD = capture;

			per_vitesseD = diff;
			VMD_mes = 1684949 / diff;
			nbImpulsionD++;

			ovfCountD = 0;   // reset overflow count moteur D
		} else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) { // Moteur gauche
			capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);

			if (capture >= lastCaptureG)
				diff = capture - lastCaptureG;
			else
				diff = (0xFFFF - lastCaptureG) + capture + 1;

			lastCaptureG = capture;

			per_vitesseG = diff;
			VMG_mes = 1684949 / diff;
			nbImpulsionG++;

			ovfCountG = 0;   // reset overflow count moteur G
		}
	}
}

/**
 * @brief Send wheel measurements to the application loop queue.
 * This function allocates memory for a WheelsState_typeDef structure,
 * populates it with the current wheel measurements, and sends it to the
 * application loop queue. If the queue is full or memory allocation fails,
 * appropriate error handling is performed.
 */
void WHEELS_SendMesures() {
	WheelsState_typeDef *wheelsState = pvPortMalloc(sizeof(WheelsState_typeDef));
	if (wheelsState != NULL) {
		wheelsState->header.id = MOTORS_MEASURES_ID;
		wheelsState->nbImpulsionG = nbImpulsionG;
		wheelsState->nbImpulsionD = nbImpulsionD;
		wheelsState->VMG_mes = VMG_mes;
		wheelsState->VMD_mes = VMD_mes;

		if (xQueueSend(xAppLoopQueue, &wheelsState, 0) != pdPASS) {
			// Queue full, drop the message
			vPortFree(wheelsState);
		}
	} else {
		// Memory allocation error
		Error_Handler();
	}
}
