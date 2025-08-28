/*
 * measures.h
 *
 *  Created on: Aug 20, 2025
 *      Author: dimercur
 */

#ifndef MEASURES_H_
#define MEASURES_H_

#include "app.h"

typedef struct {
	AppMessage_typeDef header;
	uint16_t batteryLevel; // in mV
} BatteryMeasure_typeDef;

void MEASURES_Init(void);

uint16_t MEASURES_GetBatteryLevel(void);
uint16_t MEASURES_GetMotorLeftCurrent(void);
uint16_t MEASURES_GetMotorRightCurrent(void);
uint16_t MEASURES_GetSteeringCurrent(void);
uint16_t MEASURES_GetSteeringAngle(void);

#endif /* MEASURES_H_ */
