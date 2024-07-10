#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "trames_nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "globalvar.h"

uint32_t SCHEDULER_Init(void);

void SCHEDULER_Run(void);

void task_update_gps(void);

void task_send_values_GPS (void);

#endif /* INC_SCHEDULER_H_ */
