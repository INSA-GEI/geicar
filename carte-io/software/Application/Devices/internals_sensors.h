/*
 * internals_sensors.h
 *
 *  Created on: May 28, 2025
 *      Author: dimercur
 */

#ifndef DEVICES_INTERNALS_SENSORS_H_
#define DEVICES_INTERNALS_SENSORS_H_

#include "FreeRTOS.h"

typedef struct {
	float x;
	float y;
	float z;
} Sensor_Acceleration_TypeDef;

typedef struct {
	float x;
	float y;
	float z;
} Sensor_Gyroscope_TypeDef;

typedef struct {
	float temperature;
} Sensor_Temperature_TypeDef;

typedef struct {
	float pressure;
} Sensor_Pressure_TypeDef;

typedef struct {
	float magneticFieldX;
	float magneticFieldY;
	float magneticFieldZ;
} Sensor_Magnetic_TypeDef;

typedef struct {
	uint32_t lsm6ds3tr_c 	: 1;
	uint32_t lps22df 		: 1;
	uint32_t lis2mdl 		: 1;
	uint32_t apds_9251 		: 1;
	uint32_t sht40 			: 1;
	uint32_t reserved 		:27;
} Sensor_ProbeResults_TypeDef;

void INT_SENSORS_Init(void);
void INT_SENSORS_Probe(Sensor_ProbeResults_TypeDef *probeResults);
BaseType_t INT_SENSORS_GetAcceleration(Sensor_Acceleration_TypeDef *acceleration);

#endif /* DEVICES_INTERNALS_SENSORS_H_ */
