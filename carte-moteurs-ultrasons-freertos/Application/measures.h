/**
 * @file measures.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Header file for measures.c
 * This file contains the declarations for functions to read and send various measurements such as battery level and motor currents.
 */

#ifndef __MEASURES_H__
#define __MEASURES_H__

#ifdef __cplusplus
extern "C" {
#endif

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

void MEASURES_SendBatteryLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* __MEASURES_H__ */
