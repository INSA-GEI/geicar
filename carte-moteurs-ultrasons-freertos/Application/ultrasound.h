#ifndef _ULTRASOUND_H_
#define _ULTRASOUND_H_

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

#endif /* _ULTRASOUND_H_ */
