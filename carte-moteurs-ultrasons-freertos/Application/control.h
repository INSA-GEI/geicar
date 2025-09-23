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

#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"
#include "can_communication.h"

#define DISABLED -1
#define STOP 50


void CAR_CONTROL_Init(void);

/**
*	Controle les MARG, MARD et MAV à partir de modeSpeed et modeSteer recus via le CAN
**/
void CAR_CONTROL_Manage(void);

void CAR_CONTROL_Update(int left_rear_speed, int right_rear_speed, int8_t steering_angle);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H__ */
