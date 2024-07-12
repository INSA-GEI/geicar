#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "trames_nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "globalvar.h"
#include "message.h"

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
	uint8_t gps_data_lat[8];
	uint8_t gps_data_long[8];
	uint8_t gps_data_alt[8];
}GPSFrameTypeDef;
#pragma pack(pop)

uint32_t SCHEDULER_Init(void);

void SCHEDULER_Run(void);

void task_update_gps(void);

void task_send_values_GPS (void);

void TransmitGPSFrame(GPSFrameTypeDef *frame);

#endif /* INC_SCHEDULER_H_ */
