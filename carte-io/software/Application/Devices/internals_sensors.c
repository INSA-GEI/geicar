/*
 * internals_sensors.c
 *
 *  Created on: May 28, 2025
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"
#include "internals_sensors.h"
#include "config.h"
#include "Services/i2cdrv.h"

#include "FreeRTOS.h"
#include "task.h"

// Capteurs internes
#include "Devices/lsm6ds3tr-c_STdC/lsm6ds3tr-c_reg.h"
#include "Devices/lps22df_STdC/lps22df_reg.h"
#include "Devices/lis2mdl_STdC/lis2mdl_reg.h"

////////////////////////////////////////////////////////////
// fonctions de lecture/écriture pour les capteurs internes
////////////////////////////////////////////////////////////

/**
 * @brief  Fonction de lecture d'un registre via I2C
 * @param  handle: Handle du capteur
 * @param  reg: Registre à lire
 * @param  bufp: Buffer pour stocker les données lues
 * @param  len: Longueur des données à lire
 * @retval 0 si succès, -1 en cas d'erreur
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len) {
	uint32_t i2c_adresse = (uint32_t) handle;

	if (I2C_Transmit(I2C_INTERNAL, (uint16_t) i2c_adresse, (uint8_t*) &reg, 1, 1000) != HAL_OK) {
		return -1; // Erreur de transmission
	}

	if ((bufp != NULL) && (len > 0)) {
		if (I2C_Transmit(I2C_INTERNAL, (uint16_t) i2c_adresse, (uint8_t*)bufp, (uint16_t) (len), 1000) != HAL_OK) {
			return -1; // Erreur de transmission
		}
	}

	return 0; // Succès
}

/**
 * @brief  Fonction de lecture d'un registre via I2C
 * @param  handle: Handle du capteur
 * @param  reg: Registre à lire
 * @param  bufp: Buffer pour stocker les données lues
 * @param  len: Longueur des données à lire
 * @retval 0 si succès, -1 en cas d'erreur
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	uint32_t i2c_adresse = (uint32_t) handle;

	if (I2C_Transmit(I2C_INTERNAL, (uint16_t) i2c_adresse, (uint8_t*) &reg, 1, 1000) != HAL_OK) {
		return -1; // Erreur de transmission
	}

	if ((bufp != NULL) && (len > 0)) {
		if (I2C_Receive(I2C_INTERNAL, (uint16_t) i2c_adresse, (uint8_t*)bufp, (uint16_t) (len), 1000) != HAL_OK) {
			return -1; // Erreur de transmission
		}
	}

	return 0; // Succès
}

/**
 * @brief  Fonction de temporisation
 * @param  ms: Temps en millisecondes
 * @retval None
 */
static void platform_delay(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms)); // Utilisation de FreeRTOS pour la temporisation
}

////////////////////////////////////////////////////////////
// Contextes pour les differents capteurs I2C
////////////////////////////////////////////////////////////

const stmdev_ctx_t lis2mdl_ctx = { .read_reg = platform_read, .write_reg = platform_write, .mdelay = platform_delay,
		.handle = (void*) LIS2MDL_I2C_ADD, // Utiliser le handle pour stocker l'adresse I2C du capteur
};

const stmdev_ctx_t lps22df_ctx = { .read_reg = platform_read, .write_reg = platform_write, .mdelay = platform_delay,
		.handle = (void*) LPS22DF_I2C_ADD_L, // Utiliser le handle pour stocker l'adresse I2C du capteur
};

const stmdev_ctx_t lsm6ds3tr_ctx = { .read_reg = platform_read, .write_reg = platform_write, .mdelay = platform_delay,
		.handle = (void*) LSM6DS3TR_C_I2C_ADD_L, // Utiliser le handle pour stocker l'adresse I2C du capteur
};

//////////////////////////////////////////////////////////////////////////
// Fonctions dediées à chaque capteurs
//////////////////////////////////////////////////////////////////////////

// LSM6DS3TR-C
BaseType_t INT_SENSORS_Init_lsm6ds3tr(void) {
	BaseType_t result = pdFALSE;
	uint8_t whoamI, rst;

	/* Check device ID */
	whoamI = 0;
	lsm6ds3tr_c_device_id_get(&lsm6ds3tr_ctx, &whoamI);

	if ( whoamI == LSM6DS3TR_C_ID ) {
		/* Restore default configuration */
		lsm6ds3tr_c_reset_set(&lsm6ds3tr_ctx, PROPERTY_ENABLE);

		do {
			lsm6ds3tr_c_reset_get(&lsm6ds3tr_ctx, &rst);
		} while (rst);

		/* Enable Block Data Update */
		lsm6ds3tr_c_block_data_update_set(&lsm6ds3tr_ctx, PROPERTY_ENABLE);
		/* Set Output Data Rate */
		lsm6ds3tr_c_xl_data_rate_set(&lsm6ds3tr_ctx, LSM6DS3TR_C_XL_ODR_12Hz5);
		lsm6ds3tr_c_gy_data_rate_set(&lsm6ds3tr_ctx, LSM6DS3TR_C_GY_ODR_12Hz5);
		/* Set full scale */
		lsm6ds3tr_c_xl_full_scale_set(&lsm6ds3tr_ctx, LSM6DS3TR_C_2g);
		lsm6ds3tr_c_gy_full_scale_set(&lsm6ds3tr_ctx, LSM6DS3TR_C_2000dps);
		/* Configure filtering chain(No aux interface) */
		/* Accelerometer - analog filter */
		lsm6ds3tr_c_xl_filter_analog_set(&lsm6ds3tr_ctx,
				LSM6DS3TR_C_XL_ANA_BW_400Hz);
		/* Accelerometer - LPF1 path ( LPF2 not used )*/
		//lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
		/* Accelerometer - LPF1 + LPF2 path */
		lsm6ds3tr_c_xl_lp2_bandwidth_set(&lsm6ds3tr_ctx,
				LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
		/* Accelerometer - High Pass / Slope path */
		//lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
		//lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
		/* Gyroscope - filtering chain */
		lsm6ds3tr_c_gy_band_pass_set(&lsm6ds3tr_ctx,
				LSM6DS3TR_C_HP_260mHz_LP1_STRONG);

		result = pdTRUE; // Initialisation réussie
	}

	return result;
}

// LIS2MDL
BaseType_t INT_SENSORS_Init_lis2mdl(void) {
	BaseType_t result = pdFALSE;
	uint8_t whoamI, rst;

	lis2mdl_device_id_get(&lis2mdl_ctx, &whoamI);

	if (whoamI == LIS2MDL_ID) {
		/* Restore default configuration */
		lis2mdl_reset_set(&lis2mdl_ctx, PROPERTY_ENABLE);

		do {
			lis2mdl_reset_get(&lis2mdl_ctx, &rst);
		} while (rst);

		/* Enable Block Data Update */
		lis2mdl_block_data_update_set(&lis2mdl_ctx, PROPERTY_ENABLE);
		/* Set Output Data Rate */
		lis2mdl_data_rate_set(&lis2mdl_ctx, LIS2MDL_ODR_10Hz);
		/* Set / Reset sensor mode */
		lis2mdl_set_rst_mode_set(&lis2mdl_ctx, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
		/* Enable temperature compensation */
		lis2mdl_offset_temp_comp_set(&lis2mdl_ctx, PROPERTY_ENABLE);
		/* Set device in continuous mode */
		lis2mdl_operating_mode_set(&lis2mdl_ctx, LIS2MDL_CONTINUOUS_MODE);

		result = pdTRUE; // Initialisation réussie
	}

	return result; // A implémenter si nécessaire
}

// LPS22DF
BaseType_t INT_SENSORS_Init_lps22df(void) {
	BaseType_t result = pdFALSE;
	lps22df_id_t id;
	lps22df_md_t md;
	int ret;
	lps22df_pin_int_route_t int_route;
	lps22df_bus_mode_t bus_mode;

	/* Check device ID */
	lps22df_id_get(&lps22df_ctx, &id);
	if (id.whoami == LPS22DF_ID) {

		/* Boot device */
		ret = lps22df_init_set(&lps22df_ctx, LPS22DF_BOOT);
		if (ret != 0)
			while(1);

		/* Reset device */
		ret = lps22df_init_set(&lps22df_ctx, LPS22DF_RESET);
		if (ret != 0)
			while(1);

		/* Set bdu and if_inc recommended for driver usage */
		lps22df_init_set(&lps22df_ctx, LPS22DF_DRV_RDY);

		/* Select bus interface */
		bus_mode.filter = LPS22DF_FILTER_AUTO;
		bus_mode.interface = LPS22DF_SEL_BY_HW;
		lps22df_bus_mode_set(&lps22df_ctx, &bus_mode);

		/* Set Output Data Rate */
		md.odr = LPS22DF_4Hz;
		md.avg = LPS22DF_16_AVG;
		md.lpf = LPS22DF_LPF_ODR_DIV_4;
		lps22df_mode_set(&lps22df_ctx, &md);

		/* Configure inerrupt pins */
		lps22df_pin_int_route_get(&lps22df_ctx, &int_route);
		int_route.drdy_pres   = PROPERTY_DISABLE;
		lps22df_pin_int_route_set(&lps22df_ctx, &int_route);

		result = pdTRUE; // Initialisation réussie
	}

	return result;
}

// SHT40
BaseType_t INT_SENSORS_Init_sht40(void) {
	BaseType_t result = pdFALSE;

	return result; // A implémenter si nécessaire
}

// APDS-9251
BaseType_t INT_SENSORS_Init_apds9251(void) {
	BaseType_t result = pdFALSE;

	return result; // A implémenter si nécessaire
}

/**
 * @brief  Fonction d'initialisation des capteurs internes
 * @retval None
 */
void INT_SENSORS_Init(void) {
	/*  Rien a faire ici */
}

/**
 * @brief  Fonction de sondage des capteurs internes
 * @param  probeResults: Pointeur vers les résultats du sondage
 * @retval void
 */
void INT_SENSORS_Probe(Sensor_ProbeResults_TypeDef *probeResults) {
	probeResults->lsm6ds3tr_c = 0;
	probeResults->lps22df = 0;
	probeResults->lis2mdl = 0;
	probeResults->apds_9251 = 0;
	probeResults->sht40 = 0;

	// Configuration du capteur LSM6DS3TR-C
	if (INT_SENSORS_Init_lsm6ds3tr() == pdTRUE) {
		probeResults->lsm6ds3tr_c = 1 ; // Indiquer que le capteur LSM6DS3TR-C est présent
	}

	// Configuration du capteur LPS22DF
	if (INT_SENSORS_Init_lps22df() == pdTRUE) {
		probeResults->lps22df = 1; // Indiquer que le capteur LPS22DF est présent
	}

	// Configuration du capteur LIS2MDL
	if (INT_SENSORS_Init_lis2mdl() == pdTRUE) {
		probeResults->lis2mdl = 1; // Indiquer que le capteur LIS2MDL est présent
	}

	// Configuration du capteur SHT40
	if (INT_SENSORS_Init_sht40() == pdTRUE) {
		probeResults->sht40 = 1; // Indiquer que le capteur SHT40 est présent
	}

	// Configuration du capteur APDS-9251
	if (INT_SENSORS_Init_apds9251() == pdTRUE) {
		probeResults->apds_9251 = 1; // Indiquer que le capteur APDS-9251 est présent
	}
}

/**
 * @brief  Fonction pour obtenir les données d'accélération
 * @param  acceleration: Pointeur vers la structure pour stocker les données d'accélération
 * @retval pdTRUE si la récupération des données a réussi, pdFALSE sinon
 */
BaseType_t INT_SENSORS_GetAcceleration(Sensor_Acceleration_TypeDef *acceleration) {
	BaseType_t result = pdFALSE;

	return result;
}

