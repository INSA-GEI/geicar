/*
 * control.h
 *
 *  Created on: 12 nov. 2021
 *      Author: Carole Meyer
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define DISABLED -1
#define STOP 50

/**
*	Controle les MARG, MARD et MAV à partir de modeSpeed et modeSteer recus via le CAN
**/
void CAR_CONTROL_Manage(int left_rear_speed, int right_rear_speed, int steering_angle);

#endif /* CONTROL_H_ */
