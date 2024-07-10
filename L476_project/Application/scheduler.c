#include <scheduler.h>
#include "stm32l4xx_hal.h"

extern char nmeaFrame[200];
extern int nmeaFrameValid;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

int gpsSend = 0;

uint8_t gps_data[8] = {0,0,0,0,0,0,0,0};

uint8_t IMU1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU2[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU3[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t IMU4[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int sign(float n){
	if (n>=0){
		return 0;
	}else{
		return 1;
	}
}



/*/void task_send_values_IMU (void) {

	//IMU1 : Magnetic field
	IMU1[6] = 4*sign(current_magnetic_mG.x) + 2*sign(current_magnetic_mG.y) + 1*sign(current_magnetic_mG.z);

	int mag_x = abs(current_magnetic_mG.x);
	int mag_y = abs(current_magnetic_mG.y);
	int mag_z = abs(current_magnetic_mG.z);

	IMU1[0]= (mag_x & 0xff00)>>8;
	IMU1[1]= (mag_x & 0xff);
	IMU1[2]= (mag_y & 0xff00)>>8;
	IMU1[3]= (mag_y & 0xff);
	IMU1[4]= (mag_z & 0xff00)>>8;
	IMU1[5]= (mag_z & 0xff);

	//IMU2 : Angular velocity
	IMU2[6] = 4*sign(current_angular_rate_mdps.x) + 2*sign(current_angular_rate_mdps.y) + 1*sign(current_angular_rate_mdps.z);

	int ang_x = abs(current_angular_rate_mdps.x);
	int ang_y = abs(current_angular_rate_mdps.y);
	int ang_z = abs(current_angular_rate_mdps.z);

	IMU2[0]= (ang_x & 0xff00)>>8;
	IMU2[1]= (ang_x & 0xff);
	IMU2[2]= (ang_y & 0xff00)>>8;
	IMU2[3]= (ang_y & 0xff);
	IMU2[4]= (ang_z & 0xff00)>>8;
	IMU2[5]= (ang_z & 0xff);


	//IMU3 : Linear acceleration
	IMU3[6] = 4*sign(current_acceleration_mg.x) + 2*sign(current_acceleration_mg.y) + 1*sign(current_acceleration_mg.z);

	int acc_x = abs(current_acceleration_mg.x);
	int acc_y = abs(current_acceleration_mg.y);
	int acc_z = abs(current_acceleration_mg.z);

	IMU3[0]= (acc_x & 0xff00)>>8;
	IMU3[1]= (acc_x & 0xff);
	IMU3[2]= (acc_y & 0xff00)>>8;
	IMU3[3]= (acc_y & 0xff);
	IMU3[4]= (acc_z & 0xff00)>>8;
	IMU3[5]= (acc_z & 0xff);

	//IMU4 : General (temperature, pressure, humidity)
	int temp = (int)current_temperature_degC*10;
	int pressure = current_pressure_hPa;
	int humidity = current_humidity_perc;

	IMU4[0] = (temp >> 8) & 0xff;
	IMU4[1] = temp & 0xff;
	IMU4[2] = (pressure >> 8) & 0xff;
	IMU4[3] = pressure & 0xff;
	IMU4[4] = humidity & 0xff;


}*/
void task_update_gps(void) {
	if (nmeaFrameValid){
		ProcessNMEALine(nmeaFrame,&gpsCoords, &gpsQuality);
		gpsSend = 1;
		nmeaFrameValid = 0;
	}

}

void task_send_values_GPS (void) {

	if (gpsSend){
		//Latitude
		double computing = gpsCoords.lat;
		int Lat7 = floor(computing);
		computing = 100.0*(computing-(double)Lat7);
		int Lat6 = floor(computing);
		computing = 100.0*(computing-(double)Lat6);
		int Lat5 = floor(computing);
		computing = 100.0*(computing-(double)Lat5);
		int Lat4 = floor(computing);
		computing = 100.0*(computing-(double)Lat4);
		int Lat3 = floor(computing);
		computing = 100.0*(computing-(double)Lat3);
		int Lat2 = floor(computing);
		computing = 100.0*(computing-(double)Lat2);
		int Lat1 = floor(computing);
		computing = 100.0*(computing-(double)Lat1);
		int Lat0 = floor(computing);

		gps_data[0] = Lat7;
		gps_data[1] = Lat6;
		gps_data[2] = Lat5;
		gps_data[3] = Lat4;
		gps_data[4] = Lat3;
		gps_data[5] = Lat2;
		gps_data[6] = Lat1;
		gps_data[7] = Lat0;
		HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

		//Longitude
		computing = gpsCoords.lon;
		int Lon7 = floor(computing);
		computing = 100.0*(computing-(double)Lon7);
		int Lon6 = floor(computing);
		computing = 100.0*(computing-(double)Lon6);
		int Lon5 = floor(computing);
		computing = 100.0*(computing-(double)Lon5);
		int Lon4 = floor(computing);
		computing = 100.0*(computing-(double)Lon4);
		int Lon3 = floor(computing);
		computing = 100.0*(computing-(double)Lon3);
		int Lon2 = floor(computing);
		computing = 100.0*(computing-(double)Lon2);
		int Lon1 = floor(computing);
		computing = 100.0*(computing-(double)Lon1);
		int Lon0 = floor(computing);

		gps_data[0] = Lon7;
		gps_data[1] = Lon6;
		gps_data[2] = Lon5;
		gps_data[3] = Lon4;
		gps_data[4] = Lon3;
		gps_data[5] = Lon2;
		gps_data[6] = Lon1;
		gps_data[7] = Lon0;
		HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

		//Altitude
		computing = gpsCoords.alt;
		int Alt7 = floor(computing);
		computing = 100.0*(computing-(double)Alt7);
		int Alt6 = floor(computing);
		computing = 100.0*(computing-(double)Alt6);
		int Alt5 = floor(computing);
		computing = 100.0*(computing-(double)Alt5);
		int Alt4 = floor(computing);
		computing = 100.0*(computing-(double)Alt4);
		int Alt3 = floor(computing);
		computing = 100.0*(computing-(double)Alt3);
		int Alt2 = floor(computing);
		computing = 100.0*(computing-(double)Alt2);
		int Alt1 = floor(computing);
		computing = 100.0*(computing-(double)Alt1);
		int Alt0 = floor(computing);

		gps_data[0] = Alt7;
		gps_data[1] = Alt6;
		gps_data[2] = Alt5;
		gps_data[3] = Alt4;
		gps_data[4] = Alt3;
		gps_data[5] = Alt2;
		gps_data[6] = Alt1;
		gps_data[7] = Alt0;
		HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

		gpsSend = 0;
	}


}
