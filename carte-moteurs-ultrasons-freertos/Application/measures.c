/**
 * @file measures.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Functions to read and send various measurements such as battery level and motor currents.
 */

#include "measures.h"
#include "configuration.h"

#include "adc.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "tasks.h" // for xAppLoopQueue"

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

/**
 * @brief Send the battery level to the application main loop for CAN formatting.
 * This function allocates memory for a BatteryMeasure_typeDef structure,
 * populates it with the current battery level, and sends it to the application
 * main loop via a queue. If memory allocation fails or the queue is full,
 * the message is dropped.
 */
void MEASURES_SendBatteryLevel(void) {
	// Send battery level to application main loop, for CAN formating
	BatteryMeasure_typeDef *battState = pvPortMalloc(sizeof(BatteryMeasure_typeDef));

	if (battState != NULL) {
		battState->header.id = BATTERY_MEASURE_ID;
		battState->batteryLevel = MEASURES_GetBatteryLevel();

		// Send to APP task
		if (xQueueSend(xAppLoopQueue, &battState, portMAX_DELAY) != pdPASS) {
			// Queue full, drop the message
			vPortFree(battState);
		}
	} else {
		// Memory allocation failed, drop the message
	}
}
