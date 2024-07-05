/*
 * globalvar.h
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#ifndef INC_GLOBALVAR_H_
#define INC_GLOBALVAR_H_

#include <gnss_nmea_parser.h>
#include <gnss_ubx_parser.h>
#include "iks01a2.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

extern values3d_t current_acceleration_mg;
extern values3d_t current_angular_rate_mdps;
extern values3d_t current_magnetic_mG;
extern float current_pressure_hPa;
extern float current_temperature_degC;
extern float current_humidity_perc;

extern GPS_Coords_t gpsCoords;
extern int gpsQuality;
extern ubx_nav_pvt_msg_t ubx_nav_pvt_msg;

#define COM_POLL_TIMEOUT 1000

void GLOBVAR_Init(void);
#endif /* INC_GLOBALVAR_H_ */
