/**
 * @file control.c
 * @author Carole Meyer
 * @version V1.1
 * @date 12 November 2021
 * @brief Functions to control the car's propulsion and steering.
 * This file contains the implementation of functions to manage the car's propulsion wheels and steering mechanism.
 *
 * Version 1.0 - Carole Meyer - Initial implementation
 * Version 1.1 - S. DI MERCURIO - Added CAR_CONTROL_Update function for updating control parameters, added freeRTOS support
 */

/* Includes ------------------------------------------------------------------*/

#include "steering.h"
#include "wheels.h"

#include "control.h"

int CAR_CONTROL_left_rear_speed;
int CAR_CONTROL_right_rear_speed;
int8_t CAR_CONTROL_steering_angle;

void CAR_CONTROL_Init(void) {
	CAR_CONTROL_left_rear_speed = DISABLED;
	CAR_CONTROL_right_rear_speed = DISABLED;
	CAR_CONTROL_steering_angle = 0;

	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_RESET, STOP, STOP);
	STEERING_SetAngle(CAR_CONTROL_steering_angle);
}

/**
*	Update motor speeds
**/
void CAR_CONTROL_Manage(void) {

	if (CAR_CONTROL_left_rear_speed == DISABLED && CAR_CONTROL_right_rear_speed == DISABLED) {
		//Propulsion
		WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_RESET, STOP, STOP);

	} else {
		//Propulsion
		WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_SET, CAR_CONTROL_right_rear_speed, CAR_CONTROL_left_rear_speed);
	}

	//Steering
	//STEERING_SetAngle(CAR_CONTROL_steering_angle);
}

void CAR_CONTROL_Update(int left_rear_speed, int right_rear_speed, int8_t steering_angle) {

	CAR_CONTROL_left_rear_speed = left_rear_speed;
	CAR_CONTROL_right_rear_speed = right_rear_speed;
	CAR_CONTROL_steering_angle = steering_angle;

	//Steering
	STEERING_SetAngle(steering_angle);
}

//uint8_t CAR_CONTROL_DecodeCANFrame(CANReceivedFrame_typeDef *canFrame, CarControlCmd_typeDef *carCmd) {
//	if (canFrame->can_id == CAN_ID_MOTORS_CMD && canFrame->length >= 3) {
//		carCmd->header.id = CAR_CONTROL_UPDATE_ID;
//		carCmd->left_rear_speed = (int) canFrame->data[0]; // Read left rear motor speed
//		carCmd->right_rear_speed = (int) canFrame->data[1]; // Read right rear motor speed
//		carCmd->steering_angle = (int) canFrame->data[2]; // Read steering motor speed
//
//		return 1; // Success
//	} else {
//		return 0; // Not a valid CAN frame for car control
//	}
//}
