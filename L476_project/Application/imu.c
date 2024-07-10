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

	 HTS221_HUM_GetHumidity(&Handler_hts221, &current_humidity_perc);
	 HTS221_TEMP_GetTemperature(&Handler_hts221, &current_temperature_degC);
	 LSM303AGR_ACC_GetAxes(&Handler_lsm303agr_acc, &current_acceleration_mg);
	 LSM303AGR_MAG_GetAxes(&Handler_lsm303agr_mag, &current_magnetic_mG);
	 LPS22HB_PRESS_GetPressure(&Handler_lps22hb, &current_pressure_hPa);
	 LSM6DSL_GYRO_GetAxes(&Handler_lsm6dsl, &current_angular_rate_mdps);

	 snprintf((char*)message_temp,35,"temperature = %d\r\n",(int)current_temperature_degC);
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
	 MESSAGE_SendMailbox(Appli_Mailbox, MSG_ID_IMU_MAG, NULL,message_mag);

}
