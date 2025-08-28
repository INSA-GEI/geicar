/*
 * control.h
 *
 *  Created on: 12 nov. 2021
 *      Author: Carole Meyer
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "app.h"
#include "can_communication.h"

#define DISABLED -1
#define STOP 50
//
//typedef struct {
//	AppMessage_typeDef header;
//	int left_rear_speed; // 0-100
//	int right_rear_speed; // 0-100
//	int steering_angle; // 0-100
//} CarControlCmd_typeDef;

void CAR_CONTROL_Init(void);

/**
*	Controle les MARG, MARD et MAV à partir de modeSpeed et modeSteer recus via le CAN
**/
void CAR_CONTROL_Manage(void);

void CAR_CONTROL_Update(int left_rear_speed, int right_rear_speed, int steering_angle);

#endif /* CONTROL_H_ */
