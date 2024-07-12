#include <scheduler.h>
#include "stm32l4xx_hal.h"

extern char nmeaFrame[200];
extern int nmeaFrameValid;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

int gpsSend = 0;

GPSFrameTypeDef gps_data1 = {
	.gps_data_lat = {0,0,0,0,0,0,0,0},
	.gps_data_long = {0,0,0,0,0,0,0,0},
	.gps_data_alt = {0,0,0,0,0,0,0,0}
};

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

		gps_data1.gps_data_lat[0] = Lat7;
		gps_data1.gps_data_lat[1] = Lat6;
		gps_data1.gps_data_lat[2] = Lat5;
		gps_data1.gps_data_lat[3] = Lat4;
		gps_data1.gps_data_lat[4] = Lat3;
		gps_data1.gps_data_lat[5] = Lat2;
		gps_data1.gps_data_lat[6] = Lat1;
		gps_data1.gps_data_lat[7] = Lat0;
		//HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

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

		gps_data1.gps_data_long[0] = Lon7;
		gps_data1.gps_data_long[1] = Lon6;
		gps_data1.gps_data_long[2] = Lon5;
		gps_data1.gps_data_long[3] = Lon4;
		gps_data1.gps_data_long[4] = Lon3;
		gps_data1.gps_data_long[5] = Lon2;
		gps_data1.gps_data_long[6] = Lon1;
		gps_data1.gps_data_long[7] = Lon0;

		//HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

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

		gps_data1.gps_data_alt[0] = Alt7;
		gps_data1.gps_data_alt[1] = Alt6;
		gps_data1.gps_data_alt[2] = Alt5;
		gps_data1.gps_data_alt[3] = Alt4;
		gps_data1.gps_data_alt[4] = Alt3;
		gps_data1.gps_data_alt[5] = Alt2;
		gps_data1.gps_data_alt[6] = Alt1;
		gps_data1.gps_data_alt[7] = Alt0;

		MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_GPS, NULL,&gps_data1);
		//HAL_UART_Transmit_IT(&huart4,gps_data,sizeof(gps_data));

		gpsSend = 0;
	}


}

void TransmitGPSFrame(GPSFrameTypeDef *frame) {
    // Convertir la structure en un tableau de bytes
    uint8_t buffer[sizeof(GPSFrameTypeDef)];
    memcpy(buffer, frame, sizeof(GPSFrameTypeDef));

    // Transmettre le tableau de bytes via UART
    HAL_UART_Transmit_IT(&huart4, buffer, sizeof(GPSFrameTypeDef));
}
