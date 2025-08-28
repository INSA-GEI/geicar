/**
 * @file ultrasound.h
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Header file for ultrasound.c
 * This file contains the declarations for ultrasonic sensor functions.
 */

#ifndef __ULTRASOUND_H__
#define __ULTRASOUND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"

typedef struct {
	AppMessage_typeDef header;
	uint16_t usDistance[6]; // in cm
} UltrasoundMesures_typeDef;

void US_Init(void);

void US_StartMeasurements(void);

uint16_t US_GetDistance(uint8_t channel);
//Count overflows
void US_OverflowManager();

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASOUND_H__ */
