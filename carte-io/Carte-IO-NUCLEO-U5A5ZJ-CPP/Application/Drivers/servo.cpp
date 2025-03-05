/*
 * servo.cpp
 *
 *  Created on: Feb 26, 2025
 *      Author: dimercur
 */

#include <Drivers/servo.h>
#include "stm32u5xx_ll_tim.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_bus.h"

#define DIV_CEIL(a, b) (((a) + (b) - 1) / (b))

Servo::~Servo() {
	// TODO Auto-generated destructor stub
}

void Servo::configure (TIM_HandleTypeDef* handler, uint32_t update_freq) {
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	handler_ = handler;
	TIM_TypeDef* instance = handler_->Instance;

	if (instance == TIM3) {
		/* Peripheral clock enable */
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);

		/**TIM3 GPIO Configuration
			  PE3   ------> TIM3_CH1
			  PE4   ------> TIM3_CH2
			  PE5   ------> TIM3_CH3
			  PE6   ------> TIM3_CH4
		 */
		GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
		LL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	} else { // TIM4
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
		/**TIM4 GPIO Configuration
				  PD12   ------> TIM4_CH1
				  PD13   ------> TIM4_CH2
				  PD14   ------> TIM4_CH3
				  PD15   ------> TIM4_CH4
		 */
		GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
		LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}

	/* Configuration du Timer */
	/*
	 * l'objectif est de trouver un couple Prescaler / Autoreload maximisant autoreload en 16 bits
	 * et permettant d'obtenir la fréquence update_freq souhaitée
	 */
	uint32_t clk_freq = 160000000;
	uint32_t counter = clk_freq / update_freq; // valeur complete du prescaler * arr

	uint16_t prescaler = DIV_CEIL(counter,65535); // on souhaite un ARR sur 16 bits, donc division du compteur complet par 65535
	uint16_t arr = (uint16_t) (counter/prescaler);

	TIM_InitStruct.Prescaler = prescaler;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = arr;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(instance, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(instance);
	LL_TIM_SetClockSource(instance, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(instance, LL_TIM_CHANNEL_CH1);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 0;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(instance, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(instance, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_EnablePreload(instance, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_Init(instance, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(instance, LL_TIM_CHANNEL_CH2);
	LL_TIM_OC_EnablePreload(instance, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_Init(instance, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(instance, LL_TIM_CHANNEL_CH3);
	LL_TIM_OC_EnablePreload(instance, LL_TIM_CHANNEL_CH4);
	LL_TIM_OC_Init(instance, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(instance, LL_TIM_CHANNEL_CH4);
	LL_TIM_SetTriggerOutput(instance, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(instance);
}

void Servo::configureChannel(uint8_t channel, uint16_t min, uint16_t max) {

}

void Servo::set(uint8_t channel, uint16_t val) {

}

void Servo::setCentered(uint8_t channel, int16_t val) {

}

void Servo::hwInit(TIM_HandleTypeDef *handler) {

}

void Servo::hwDeInit(TIM_HandleTypeDef *handler) {

}

