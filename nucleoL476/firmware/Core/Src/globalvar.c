/*
 * globalvar.c
 *
 *  Created on: Sep 23, 2021
 *      Author: dimercur
 */

#include "globalvar.h"

/******************************************
 * Global sensor data
 *
 * last acquired values stored here
 */

values3d_t current_acceleration_mg;
values3d_t current_angular_rate_mdps;
values3d_t current_magnetic_mG;
float current_pressure_hPa;
float current_temperature_degC;
float current_humidity_perc;

GPS_Coords_t gpsCoords;
int gpsQuality;
ubx_nav_pvt_msg_t ubx_nav_pvt_msg;

/*
 * @brief  Initialization of global variables
 *
 */
void GLOBVAR_Init(void) {
	current_pressure_hPa=0.0;
	current_temperature_degC=0.0;
	current_humidity_perc=0.0;

	current_acceleration_mg.x=0.0;
	current_acceleration_mg.y=0.0;
	current_acceleration_mg.z=0.0;

	current_angular_rate_mdps.x=0.0;
	current_angular_rate_mdps.y=0.0;
	current_angular_rate_mdps.z=0.0;

	current_magnetic_mG.x=0.0;
	current_magnetic_mG.y=0.0;
	current_magnetic_mG.z=0.0;

	gpsCoords.alt=0.0;
	gpsCoords.lat=0.0;
	gpsCoords.lon=0.0;

}
