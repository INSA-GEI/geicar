/*
 * apds-9251.c
 *
 *  Created on: Mar 10, 2025
 *      Author: dimercur
 */

#include <Devices/apds_9251/apds_9251.h>

int32_t apds_9251_reset(const stmdev_ctx_t *ctx) {
	if (!ctx || !ctx->write_reg) return -1;

	uint8_t val = APDS_9251_SW_RESET;
	return ctx->write_reg(ctx->handle, APDS_9251_MAIN_CTRL_REG, &val, 1);
}

int32_t apds_9251_init(const stmdev_ctx_t *ctx, uint8_t mode) {
	if (!ctx || !ctx->write_reg || !ctx->read_reg) return -1;
	if ((mode != APDS_9251_MODE_ALS) && (mode != APDS_9251_MODE_COLOR)) return -1;

	uint8_t val;

	ctx->read_reg(ctx->handle, APDS_9251_MAIN_CTRL_REG, &val, 1);
	val&= ~APDS_9251_MODE_COLOR; // effacement du bit CS_mode
	val|= mode | APDS_9251_LS_ENABLE;

	return ctx->write_reg(ctx->handle, APDS_9251_MAIN_CTRL_REG, &val, 1);
}

int32_t apds_9251_get_conf(const stmdev_ctx_t *ctx, uint8_t *conf) {
	if (!ctx || !ctx->read_reg) return -1;

	return ctx->read_reg(ctx->handle, APDS_9251_MAIN_CTRL_REG, conf, 1);
}

int32_t apds_9251_set_resolution(const stmdev_ctx_t *ctx, uint8_t res) {
	if (!ctx || !ctx->write_reg || !ctx->read_reg) return -1;

	uint8_t val;

	ctx->read_reg(ctx->handle, APDS_9251_LS_MEAS_RATE, &val, 1);
	val&= ~(0x7<<4); // effacement des bits resolution
	val|= res;

	return ctx->write_reg(ctx->handle, APDS_9251_LS_MEAS_RATE, &val, 1);
}

int32_t apds_9251_set_rate(const stmdev_ctx_t *ctx, uint8_t rate) {
	if (!ctx || !ctx->write_reg || !ctx->read_reg) return -1;

	uint8_t val;

	ctx->read_reg(ctx->handle, APDS_9251_LS_MEAS_RATE, &val, 1);
	val&= ~(0x7<<0); // effacement des bits rate
	val|= rate;

	return ctx->write_reg(ctx->handle, APDS_9251_LS_MEAS_RATE, &val, 1);
}

int32_t apds_9251_get_status(const stmdev_ctx_t *ctx, uint8_t *status) {
	if (!ctx || !ctx->read_reg) return -1;

	return ctx->read_reg(ctx->handle, APDS_9251_MAIN_STATUS, status, 1);
}

int32_t apds_9251_get_ambiant_light(const stmdev_ctx_t *ctx, APDS_9251_Als_TypeDef *als) {
	if (!ctx || !ctx->read_reg || !als) return -1;

	uint8_t val[23];

	if (!ctx->read_reg(ctx->handle, APDS_9251_LS_DATA_IR_0, val, 2*3)) {
		als->ambiant_light = (val[5]<<16) | (val[4]<<8) | val[3];
		als->ir = (val[2]<<16) | (val[1]<<8) | val[0];
	} else return -2;

	return 0;
}

int32_t apds_9251_get_rgb_ir(const stmdev_ctx_t *ctx, APDS_9251_Rgb_TypeDef *color) {
	if (!ctx || !ctx->read_reg || !color) return -1;

	uint8_t val[4*3];

	if (!ctx->read_reg(ctx->handle, APDS_9251_LS_DATA_IR_0, val, 4*3)){
		color->ir = (val[2]<<16) | (val[1]<<8) | val[0];
		color->g = (val[5]<<16) | (val[4]<<8) | val[3];
		color->b = (val[8]<<16) | (val[7]<<8) | val[6];
		color->r = (val[11]<<16) | (val[10]<<8) | val[9];
	} else return -2;

	return 0;
}

int32_t apds_9251_get_part_id(const stmdev_ctx_t *ctx, uint8_t *id) {
	if (!ctx || !ctx->read_reg || !id) return -1;
	return ctx->read_reg(ctx->handle, APDS_9251_PART_ID, id, 1);
}
