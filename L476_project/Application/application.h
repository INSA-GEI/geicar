#ifndef APPLICATION_H_
#define APPLICATION_H_

#include "trames_nmea.h"
#include "imu.h"
#include "globalvar.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lidar.h"
#include <stm32l4xx_hal.h>

void Tasks_Init(void);
void StartSPI(void const * argument);
void StartUart(void const * argument);
void StartLidar(void const * argument);
void StartIMU(void const * argument);
void StartI2C(void const * argument);
void StartGPS(void const * argument);
void StartBatterie(void const * argument);
void StartCAN(void const * argument);

#endif
