/*
 * servo.h
 *
 *  Created on: Feb 26, 2025
 *      Author: dimercur
 */

#ifndef DRIVERS_SERVO_H_
#define DRIVERS_SERVO_H_

#include "stm32u5xx.h"

class Servo {
public:
	Servo() = default;
	~Servo();

	void configure (TIM_HandleTypeDef* handler, uint32_t update_freq);
	void configureChannel(uint8_t channel, uint16_t min, uint16_t max);

	void set(uint8_t channel, uint16_t val);
	void setCentered(uint8_t channel, int16_t val);
private:
	struct {
		uint16_t min;
		uint16_t max;
	} range[4];

	TIM_HandleTypeDef* handler_;

	static void hwInit(TIM_HandleTypeDef *handler);
	static void hwDeInit(TIM_HandleTypeDef *handler);
};

#endif /* DRIVERS_SERVO_H_ */
