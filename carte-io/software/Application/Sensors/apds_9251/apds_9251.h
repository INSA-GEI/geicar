/*
 * apds_9251.h
 *
 *  Created on: Mar 10, 2025
 *      Author: dimercur
 */

#ifndef APDS_9251_H_
#define APDS_9251_H_

#include <stdint.h>

/* Structure pour interface avec l'I2C */
typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t);

typedef struct {
  stmdev_write_ptr write_reg;
  stmdev_read_ptr read_reg;
  stmdev_mdelay_ptr mdelay;
  void *handle;
} stmdev_ctx_t;

typedef struct {
	uint32_t ambiant_light;
	uint32_t ir;
} APDS_9251_Als_TypeDef;

typedef struct {
	uint32_t r;
	uint32_t g;
	uint32_t b;
	uint32_t ir;
} APDS_9251_Rgb_TypeDef;

#define APDS_9251_I2C_ADDR  (0x52<<1)  // Adresse I2C par défaut du capteur

/* Registres du capteur */
#define APDS_9251_MAIN_CTRL_REG		0x00
#define APDS_9251_LS_MEAS_RATE  	0x04
#define APDS_9251_LS_GAIN 			0x05
#define APDS_9251_PART_ID 			0x06
#define APDS_9251_MAIN_STATUS 		0x07
#define APDS_9251_LS_DATA_IR_0 		0x0A
#define APDS_9251_LS_DATA_IR_1 		0x0B
#define APDS_9251_LS_DATA_IR_2 		0x0C
#define APDS_9251_AMBIANT_LIGHT_0 	0x0D //
#define APDS_9251_AMBIANT_LIGHT_1 	0x0E //
#define APDS_9251_AMBIANT_LIGHT_2 	0x0F // Ambiant light sensor share registers
#define APDS_9251_LS_DATA_GREEN_0 	0x0D // with green color sensor registers
#define APDS_9251_LS_DATA_GREEN_1 	0x0E //
#define APDS_9251_LS_DATA_GREEN_2 	0x0F //
#define APDS_9251_LS_DATA_BLUE_0 	0x10
#define APDS_9251_LS_DATA_BLUE_1 	0x11
#define APDS_9251_LS_DATA_BLUE_2 	0x12
#define APDS_9251_LS_DATA_RED_0 	0x13
#define APDS_9251_LS_DATA_RED_1 	0x14
#define APDS_9251_LS_DATA_RED_2 	0x15
#define APDS_9251_INT_CFG 			0x19
#define APDS_9251_INT_PERSISTENCE 	0x1A
#define APDS_9251_LS_THRES_UP_0 	0x21
#define APDS_9251_LS_THRES_UP_1 	0x22
#define APDS_9251_LS_THRES_UP_2 	0x23
#define APDS_9251_LS_THRES_LOW_0 	0x24
#define APDS_9251_LS_THRES_LOW_1 	0x25
#define APDS_9251_LS_THRES_LOW_2 	0x26
#define APDS_9251_LS_THRES_VAR  	0x27

// constants for MAIN_CTRL register
#define APDS_9251_LS_ENABLE			(0x01<<0)
#define APDS_9251_MODE_COLOR 		(0x01<<2)
#define APDS_9251_MODE_ALS 			(0x00<<2)
#define APDS_9251_SW_RESET			(0x01<<4)

// constants for LS_MEAS_RATE register
#define APDS_9251_RES_20BITS		(0x00<<4)
#define APDS_9251_RES_19BITS		(0x01<<4)
#define APDS_9251_RES_18BITS		(0x02<<4)
#define APDS_9251_RES_17BITS		(0x03<<4)
#define APDS_9251_RES_16BITS		(0x04<<4)
#define APDS_9251_RES_13BITS		(0x05<<4)

#define APDS_9251_RATE_25MS			(0x00<<0)
#define APDS_9251_RATE_50MS			(0x01<<0)
#define APDS_9251_RATE_100MS		(0x02<<0)
#define APDS_9251_RATE_200MS		(0x03<<0)
#define APDS_9251_RATE_500MS		(0x04<<0)
#define APDS_9251_RATE_1000MS		(0x05<<0)
#define APDS_9251_RATE_2000MS		(0x06<<0)

// constants for LS_GAIN register
#define APDS_9251_GAIN_1			(0x00<<0)
#define APDS_9251_GAIN_3			(0x01<<0)
#define APDS_9251_GAIN_6			(0x02<<0)
#define APDS_9251_GAIN_9			(0x03<<0)
#define APDS_9251_GAIN_18			(0x04<<0)

// constants for PART_ID register
#define APDS_9251_WHOAMI			0xB5

// constants for MAIN_STATUS register
#define APDS_9251_STATUS_POWER_ON	(0x01<<5)
#define APDS_9251_STATUS_INT		(0x01<<4)
#define APDS_9251_STATUS_DATA		(0x01<<3)

/* Fonctions API */
int32_t apds_9251_reset(const stmdev_ctx_t *ctx);
int32_t apds_9251_init(const stmdev_ctx_t *ctx, uint8_t mode);
int32_t apds_9251_get_conf(const stmdev_ctx_t *ctx, uint8_t *conf);

int32_t apds_9251_set_resolution(const stmdev_ctx_t *ctx, uint8_t res);
int32_t apds_9251_set_rate(const stmdev_ctx_t *ctx, uint8_t rate);
int32_t apds_9251_get_status(const stmdev_ctx_t *ctx, uint8_t *status);

int32_t apds_9251_get_ambiant_light(const stmdev_ctx_t *ctx, APDS_9251_Als_TypeDef *als);
int32_t apds_9251_get_rgb_ir(const stmdev_ctx_t *ctx, APDS_9251_Rgb_TypeDef *color);
int32_t apds_9251_get_part_id(const stmdev_ctx_t *ctx, uint8_t *id);

#endif /* APDS_9251_H */

