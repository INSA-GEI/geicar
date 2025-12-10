/*
 * i2c_sensors.h
 *
 *  Created on: Mar 6, 2025
 *      Author: dimercur
 */

#ifndef DEVICES_I2C_SENSORS_H_
#define DEVICES_I2C_SENSORS_H_

#include "stm32u5xx_hal.h"
#include "Drivers/i2cdriver.h"

class I2CSensors {
public:
	I2CSensors() = default;
	I2CSensors(I2cDriver *i2cdrv, uint8_t i2caddr) :driver(i2cdrv), addr(i2caddr) {}
	virtual ~I2CSensors();

    I2cDriver *driver;  // Chaque classe aura sa propre version
    uint8_t addr;
};

#endif /* DEVICES_I2C_SENSORS_H_ */
