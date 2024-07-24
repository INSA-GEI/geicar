#include <imu.h>

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

HTS221_IO_t hts221_io;
HTS221_Object_t Handler_hts221;

LSM303AGR_IO_t lsm303agr_io;
LSM303AGR_ACC_Object_t Handler_lsm303agr_acc;
LSM303AGR_MAG_Object_t Handler_lsm303agr_mag;

LSM6DSL_IO_t lsm6dls_io;
LSM6DSL_Object_t Handler_lsm6dsl;

LPS22HB_IO_t lps22hb_io;
LPS22HB_Object_t Handler_lps22hb;

uint8_t hts221_id;
uint8_t lsm6dsl_id;
uint8_t lsm303agr_id;
uint8_t lps22hb_id;

//LSM6DSL_Axes_t Acc_lsm6dsl;
LSM6DSL_Axes_t Gyro;

LSM303AGR_Axes_t Acc_lsm303agr;
LSM303AGR_Axes_t Mag_lsm303agr;

float pression;
float humidite;
float temperature;

IMUFrameTypeDef imu_frame;

/*uint8_t CalculateCRC(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}*/


static const uint8_t CrcTable[256] =
{
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

uint8_t CalculateCRC(uint8_t package[], uint8_t len)
{
	uint8_t crc = 0;
	uint16_t i;
	for (i = 0; i < len; i++)
	{
		crc += package[i];
		//crc = CrcTable[(crc ^ package[i]) & 0xff];
	}
	return 0xFF - crc;
}

void IMU_init(void)
{
	hts221_io.Address = HTS221_I2C_ADDRESS;
	hts221_io.BusType = HTS221_I2C_BUS;
	hts221_io.Init = CUSTOM_HTS221_0_I2C_Init;
	hts221_io.DeInit = CUSTOM_HTS221_0_I2C_DeInit;
	hts221_io.ReadReg = CUSTOM_HTS221_0_I2C_ReadReg;
	hts221_io.WriteReg = CUSTOM_HTS221_0_I2C_WriteReg;
	hts221_io.GetTick = (int32_t (*)(void)) HAL_GetTick;
	hts221_io.Delay = HAL_Delay;

	/* Init hts221 */
	if (HTS221_RegisterBusIO(&Handler_hts221, &hts221_io)!=0) {
		while(1);
	}

	if (HTS221_Init(&Handler_hts221)!=0) {
		while (1);
	}

	lsm303agr_io.Address = LSM303AGR_I2C_ADD_XL;
	lsm303agr_io.BusType = LSM303AGR_I2C_BUS;
	lsm303agr_io.Init = CUSTOM_HTS221_0_I2C_Init;
	lsm303agr_io.DeInit = CUSTOM_HTS221_0_I2C_DeInit;
	lsm303agr_io.ReadReg = CUSTOM_HTS221_0_I2C_ReadReg;
	lsm303agr_io.WriteReg = CUSTOM_HTS221_0_I2C_WriteReg;
	lsm303agr_io.GetTick = (int32_t (*)(void)) HAL_GetTick;
	lsm303agr_io.Delay = HAL_Delay;

	/* Init lsm303agr */
	if (LSM303AGR_ACC_RegisterBusIO(&Handler_lsm303agr_acc, &lsm303agr_io)!=0) {
		while(1);
	}

	lsm303agr_io.Address = LSM303AGR_I2C_ADD_MG;
	if (LSM303AGR_MAG_RegisterBusIO(&Handler_lsm303agr_mag, &lsm303agr_io)!=0) {
		while(1);
	}

	if (LSM303AGR_ACC_Init(&Handler_lsm303agr_acc)!=0) {
		while (1);
	}

	if (LSM303AGR_MAG_Init(&Handler_lsm303agr_mag)!=0) {
		while (1);
	}


	lsm6dls_io.Address = LSM6DSL_I2C_ADD_H;
	lsm6dls_io.BusType = LSM6DSL_I2C_BUS;
	lsm6dls_io.Init = CUSTOM_HTS221_0_I2C_Init;
	lsm6dls_io.DeInit = CUSTOM_HTS221_0_I2C_DeInit;
	lsm6dls_io.ReadReg = CUSTOM_HTS221_0_I2C_ReadReg;
	lsm6dls_io.WriteReg = CUSTOM_HTS221_0_I2C_WriteReg;
	lsm6dls_io.GetTick = (int32_t (*)(void)) HAL_GetTick;
	lsm6dls_io.Delay = HAL_Delay;

	/* Init LSM6DSL */
	if (LSM6DSL_RegisterBusIO(&Handler_lsm6dsl, &lsm6dls_io)!=0) {
		while(1);
	}

	if (LSM6DSL_Init(&Handler_lsm6dsl)!=0) {
		while (1);
	}


	lps22hb_io.Address = LPS22HB_I2C_ADD_H;
	lps22hb_io.BusType = LPS22HB_I2C_BUS;
	lps22hb_io.Init = CUSTOM_HTS221_0_I2C_Init;
	lps22hb_io.DeInit = CUSTOM_HTS221_0_I2C_DeInit;
	lps22hb_io.ReadReg = CUSTOM_HTS221_0_I2C_ReadReg;
	lps22hb_io.WriteReg = CUSTOM_HTS221_0_I2C_WriteReg;
	lps22hb_io.GetTick = (int32_t (*)(void)) HAL_GetTick;
	lps22hb_io.Delay = HAL_Delay;

	/* Init lps22hb */
	if (LPS22HB_RegisterBusIO(&Handler_lps22hb, &lps22hb_io)!=0) {
		while(1);
	}

	if (LPS22HB_Init(&Handler_lps22hb)!=0) {
		while (1);
	}



}

void IMU_enable(void)
{
	if(HTS221_HUM_Enable(&Handler_hts221)!=0){
		while (1);
	}

	if(HTS221_TEMP_Enable(&Handler_hts221)!=0){
		while (1);
	}

	if (LSM303AGR_ACC_Enable(&Handler_lsm303agr_acc)!=0) {
		while (1);
	}

	if (LSM303AGR_MAG_Enable(&Handler_lsm303agr_mag)!=0) {
		while (1);
	}

	if (LSM6DSL_ACC_Enable(&Handler_lsm6dsl)!=0) {
		while (1);
	}

	if (LSM6DSL_GYRO_Enable(&Handler_lsm6dsl)!=0) {
		while (1);
	}
	if (LPS22HB_PRESS_Enable(&Handler_lps22hb)!=0) {
		while (1);
	}
}

void IMU_GetData(void)
{
	 //IMUFrameTypeDef imu_frame;

	 HTS221_HUM_GetHumidity(&Handler_hts221, &current_humidity_perc);
	 HTS221_TEMP_GetTemperature(&Handler_hts221, &current_temperature_degC);
	 LSM303AGR_ACC_GetAxes(&Handler_lsm303agr_acc, &current_acceleration_mg);
	 LSM303AGR_MAG_GetAxes(&Handler_lsm303agr_mag, &current_magnetic_mG);
	 LPS22HB_PRESS_GetPressure(&Handler_lps22hb, &current_pressure_hPa);
	 LSM6DSL_GYRO_GetAxes(&Handler_lsm6dsl, &current_angular_rate_mdps);

	 imu_frame.humidity = current_humidity_perc;
	 imu_frame.temperature = current_temperature_degC;
	 imu_frame.pressure = current_pressure_hPa;
	 imu_frame.magnetic_x = current_magnetic_mG.x;
	 imu_frame.magnetic_y = current_magnetic_mG.y;
	 imu_frame.magnetic_z = current_magnetic_mG.z;
	 imu_frame.acceleration_x = current_acceleration_mg.x;
	 imu_frame.acceleration_y = current_acceleration_mg.y;
	 imu_frame.acceleration_z = current_acceleration_mg.z;
	 imu_frame.gyro_x = current_angular_rate_mdps.x;
	 imu_frame.gyro_y = current_angular_rate_mdps.y;
	 imu_frame.gyro_z = current_angular_rate_mdps.z;

	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU, NULL,&imu_frame);

	 /*snprintf((char*)message_temp,35,"temperature = %d\r\n",(int)current_temperature_degC);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_TEMP, NULL,message_temp);
	 snprintf((char*)message_hum,35,"humidite = %d\r\n",(int)current_humidity_perc);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_HUM, NULL,message_hum);
	 snprintf((char*)message_press,35,"pression = %d\r\n",(int)current_pressure_hPa);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_PRESS, NULL,message_press);
	 snprintf((char*)message_acc1,50,"accelerometre : x =%d y=%d z=%d\r\n", (int)current_acceleration_mg.x, (int)current_acceleration_mg.y, (int)current_acceleration_mg.z);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_ACC, NULL,message_acc1);
	 snprintf((char*)message_gyro,50,"gyroscope : x =%d y=%d z=%d\r\n",(int)current_angular_rate_mdps.x, (int)current_angular_rate_mdps.y, (int)current_angular_rate_mdps.z);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_GYR, NULL,message_gyro);
	 snprintf((char*)message_mag,50,"magnétomètre : x =%d y=%d z=%d\r\n",(int)current_magnetic_mG.x, (int)current_magnetic_mG.y, (int)current_magnetic_mG.z);
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_MAG, NULL,message_mag);*/

}

void TransmitIMUFrame(IMUFrameTypeDef *frame) {
    // Convertir la structure en un tableau de bytes
	uint8_t *buffer = (uint8_t *)malloc(sizeof(API_FrameTypeDef_IMU));
    //uint8_t buffer[sizeof(API_FrameTypeDef_IMU)];
	if (buffer == NULL) {
		// Gérer l'erreur d'allocation de mémoire
		return;
	}
    API_FrameTypeDef_IMU api_frame = {
    .header = API_HEADER,
    .length = sizeof(IMUFrameTypeDef),
    .frame_type = MSG_ID_IMU,
	.data={0},
	.crc = 0};

    memcpy(&(api_frame.data), frame, sizeof(IMUFrameTypeDef));
    api_frame.crc = CalculateCRC((uint8_t*)&(api_frame.data), sizeof(IMUFrameTypeDef));
    memcpy(buffer,&api_frame, sizeof(API_FrameTypeDef_IMU));

    adresse_buffer = buffer;
    // Transmettre le tableau de bytes via UART
    HAL_UART_Transmit_IT(&huart4, buffer, sizeof(API_FrameTypeDef_IMU));
}
