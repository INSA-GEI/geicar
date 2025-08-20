#ifndef _ULTRASOUND_H_
#define _ULTRASOUND_H_

#include "stm32f1xx_hal.h"
//#include <math.h>

void US_Init(void);

uint16_t US_GetDistance(uint8_t channel);
//Count overflows
void US_OverflowManager();

#endif /* _ULTRASOUND_H_ */
