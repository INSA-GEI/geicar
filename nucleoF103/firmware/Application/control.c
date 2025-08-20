/*
 * control.c
 *
 *  Created on: 12 nov. 2021
 *      Author: Carole Meyer
 */


/* Includes ------------------------------------------------------------------*/

#include "steering.h"
#include "wheels.h"

#include "control.h"

/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Programs ------------------------------------------------------------------*/

/**
*	Update motor speeds
**/
void CAR_CONTROL_Manage(int left_rear_speed, int right_rear_speed, int steering_speed) {

	if (left_rear_speed == DISABLED && right_rear_speed == DISABLED && steering_speed == DISABLED) {
		//Propulsion
		WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_RESET, STOP, STOP);

		//Steering
		STEERING_SetSpeed(GPIO_PIN_RESET, STOP);
	} else {
		//Propulsion
		WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_SET, right_rear_speed, left_rear_speed);

		//Steering
		STEERING_SetSpeed(GPIO_PIN_SET, steering_speed);
	}
}

