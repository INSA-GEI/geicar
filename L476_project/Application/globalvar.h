#ifndef INC_GLOBALVAR_H_
#define INC_GLOBALVAR_H_

#include "trames_nmea.h"
#include "lsm303agr.h"
#include "lsm6dsl.h"
#include "stm32l4xx_hal.h"
#include "imu.h"
//#include "iks01a2.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

extern LSM303AGR_Axes_t current_acceleration_mg;
extern LSM6DSL_Axes_t current_angular_rate_mdps;
extern LSM303AGR_Axes_t current_magnetic_mG;
extern float current_pressure_hPa;
extern float current_temperature_degC;
extern float current_humidity_perc;

extern uint8_t messages[50];
extern uint8_t message_index;

extern uint8_t message_press[35];
extern uint8_t message_hum[35];
extern uint8_t message_temp[35];
extern uint8_t message_acc1[50];
extern uint8_t message_acc2[50];
extern uint8_t message_gyro[50];
extern uint8_t message_mag[50] ;

extern uint8_t *adresse_buffer ;

extern GPS_Coords_t gpsCoords;
extern int gpsQuality;
//extern ubx_nav_pvt_msg_t ubx_nav_pvt_msg;

#define COM_POLL_TIMEOUT 1000

void GLOBVAR_Init(void);

#endif /* INC_GLOBALVAR_H_ */
