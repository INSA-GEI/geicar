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

void IMU_init(void);

void IMU_enable(void);

void IMU_GetData(void);

#endif
