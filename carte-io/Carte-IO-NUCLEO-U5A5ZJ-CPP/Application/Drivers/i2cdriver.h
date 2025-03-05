/*
 * uart_drv.h
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stm32u5xx.h"

#include "FreeRTOS.h"
#include "semphr.h"

class I2cDriver {
public:
	typedef enum {
		MODE_POLLING = 0x00U, MODE_IRQ
	} I2cDriver_ModeTypeDef;

	// Constructeur par défaut
	I2cDriver() = default;
	// Destructeur
	~I2cDriver();

	// Configuration des bus I2C
	bool configure(I2C_HandleTypeDef* hi2c_internal,
			I2C_HandleTypeDef* hi2c_external,
			I2C_HandleTypeDef* hi2c_arbitrary,
			I2cDriver_ModeTypeDef mode,
			uint32_t timeout);

	bool write(uint16_t addr, uint16_t length, uint8_t *data);

	bool read(uint16_t addr, uint16_t length, uint8_t *data);

	bool regWrite(uint16_t addr,
			uint16_t reg_addr,
			uint16_t length,
			uint8_t* data);

	bool regRead(uint16_t addr,
			uint16_t reg_addr,
			uint16_t length,
			uint8_t* data);
private:
	I2C_HandleTypeDef *handler_;
	I2cDriver_ModeTypeDef mode_;
	uint32_t timeout_;

	bool deleteBufferAfterTX_;
	const uint8_t* txBuffer_;

	SemaphoreHandle_t txCompleteSemaphore_;
	SemaphoreHandle_t rxCompleteSemaphore_;

	void onTXEvent();
	void onRXEvent();
	void onErrorEvent();
	void onHWInitEvent();
	void onHWDeInitEvent();

	// Callbacks statiques pour HAL
	static void txCompleteCallback(I2C_HandleTypeDef *handler);
	static void rxCompleteCallback(I2C_HandleTypeDef *handler);
	static void errorCallback(I2C_HandleTypeDef *handler);
	static void hwInit(I2C_HandleTypeDef *handler);
	static void hwDeInit(I2C_HandleTypeDef *handler);
};

#endif // I2C_DRIVER_H
