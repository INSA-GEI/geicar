/*
 * measures.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dimercur
 */

#include "adc.h"
#include "measures.h"

/* Tous ADC sur 12 bits pleine echelle 3.3V
 ADCBUF[0] mesure batterie
 ADCBUF[1] angle direction
 ADCBUF[2] I moteur arriere gauche
 ADCBUF[3] I moteur arriere droit
 ADCBUF[4] I moteur avant
 */
uint16_t ADCBUF[5];

void MEASURES_Init(void) {
	/* Calibration of ADC1 (Important on STM32F1 */
	HAL_ADCEx_Calibration_Start(&hadc1);

	/* Start ADC in DMA mode */
	HAL_ADC_Start_DMA (&hadc1, (uint32_t *)ADCBUF, 5);
}

uint16_t MEASURES_GetBatteryLevel(void) {
	return (uint16_t)(ADCBUF[0]); // return raw battery level
}

uint16_t MEASURES_GetMotorLeftCurrent(void) {
	return (uint16_t) (ADCBUF[2]); // return raw current of left rear motor
}

uint16_t MEASURES_GetMotorRightCurrent(void) {
	return (uint16_t) (ADCBUF[3]); // return raw current of right rear motor
}

uint16_t MEASURES_GetSteeringCurrent(void) {
	return (uint16_t) (ADCBUF[4]); // return raw current of steering motor
}

uint16_t MEASURES_GetSteeringAngle(void) {
	return (uint16_t) (ADCBUF[1]); // return raw steering angle
}
