#ifndef IMU_H
#define IMU_H

#include <stdio.h>
#include <stdlib.h>
#include "custom_mems_conf.h"
#include "custom_mems_conf_app.h"
#include "hts221_reg.h"
#include "hts221.h"

#include "lsm303agr_reg.h"
#include "lsm303agr.h"

#include "lps22hb.h"
#include "lps22hb_reg.h"

#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"
#include "globalvar.h"
#include "message.h"

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
	float temperature;
	float humidity;
	float pressure;
	int32_t magnetic_x;
	int32_t magnetic_y;
	int32_t magnetic_z;
	int32_t acceleration_x;
	int32_t acceleration_y;
	int32_t acceleration_z;
	int32_t gyro_x;
	int32_t gyro_y;
	int32_t gyro_z;

}IMUFrameTypeDef;
#pragma pack(pop)

void IMU_init(void);

void IMU_enable(void);

void IMU_GetData(void);

void TransmitIMUFrame(IMUFrameTypeDef *frame);

#endif
