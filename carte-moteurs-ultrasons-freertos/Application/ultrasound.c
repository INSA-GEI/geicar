/**
 * @file ultrasound.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 * @brief Functions to control the ultrasonic sensors of the car.
 * This file contains the functions to trigger the ultrasonic sensors and measure the distance.
 * It uses TIM3 for microsecond timing and GPIO for triggering the sensors.
 */

#include "main.h"

#include "configuration.h"

#include "tim.h"
#include "ultrasound.h"

// Variables pour les mesures US
uint8_t usEchoStart;
uint8_t usEchoReceived;
//uint64_t usEchoRisingTime = ;
uint32_t usEchoDuration;
uint32_t ustimerOverflow;

uint16_t usTriggerPin[6] = {US_Front_Left_Trig_Pin,US_Front_Center_Trig_Pin,US_Front_Right_Trig_Pin,US_Rear_Left_Trig_Pin,US_Rear_Center_Trig_Pin,US_Rear_Right_Trig_Pin};

//return current time in microsecond

void US_ResetTimer(void) {
	TIM3->CNT = 0;
	ustimerOverflow = 0;
}

uint32_t US_MicroSecond(){
	//microSecondTime = ustimerOverflow * TIM3->ARR + TIM3->CNT;
	uint32_t microSecondTime = (ustimerOverflow<<16) + TIM3->CNT;
	return microSecondTime;
}

void US_MicroDelay(uint32_t delay) {
	volatile uint32_t cnt=(delay*6)+5;

	while (cnt >0) {
		cnt--;
	}
}

//Count overflows
void US_OverflowManager(){
	ustimerOverflow ++;
}

/**
 * @brief Initialize the ultrasonic sensors.
 * This function configures the necessary peripherals for the ultrasonic sensors,
 * including GPIO and Timer 3 for microsecond timing.
 */
void US_Init(void) {
	/* Enable overflow interrupt for Timer 3 */
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	/* Start Timer 3 */
	__HAL_TIM_ENABLE(&htim3);
}

void US_StartMeasurements(void) {

	for (uint8_t i = 0; i < 6; i++) {
		usDistance[i] = US_GetDistance(i);
		//HAL_Delay(50); // wait between two measurements
	}

	UltrasoundMesures_typeDef *usMsg = (UltrasoundMesures_typeDef*) pvPortMalloc(sizeof(UltrasoundMesures_typeDef));
	if (usMsg != NULL) {
		usMsg->header.id = CAN_SEND_US_ID;
		for (uint8_t i = 0; i < 6; i++) {
			usMsg->usDistance[i] = usDistance[i];
		}
		xQueueSend(xAppLoopQueue, &usMsg, portMAX_DELAY);
	} else {
		// Memory allocation error
		Error_Handler();
	}

	// Send to APP task
	if (xQueueSend(xAppLoopQueue, &usMsg, 0) != pdPASS) {
		// Queue full, drop the message
		vPortFree((void*)usMsg);
	}
}

uint16_t US_GetDistance(uint8_t channel) {
	uint16_t distance=0;

	usEchoStart = 1;

	HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[channel], GPIO_PIN_SET); //Trigger ON
	US_MicroDelay(10);
	HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[channel], GPIO_PIN_RESET); //Trigger OFF

	//HAL_Delay(40);	// Waiting to receive the echo (40 ms max for 7m distance)
	// TODO : reprendre avec une attente sur semaphore limité dans le temps
	vTaskDelay(pdMS_TO_TICKS(40)); // Waiting to receive the echo (40 ms max for 7m distance)

	if (usEchoReceived)	//If we received the echo
		distance = usEchoDuration/58;
	else //If the echo is not received (i.e. sensor failure)
		distance = 1000;	//Set distance value out of range

	usEchoReceived = 0;

	return distance;
}

// EXTI External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (usEchoStart) { //check pin state : rising
		US_ResetTimer();
		//usEchoRisingTime = 0;
		usEchoStart = 0;
	} else { // Mesure falling edge
		// uint64_t echoDuration = microSecond() - usEchoRisingTime;
		//uint64_t echoDuration = US_MicroSecond();

		/*if (echoDuration >= 0)
			usEchoDuration = echoDuration;
		else
			usEchoDuration=-1;*/

		//usEchoStart = 1;
		usEchoDuration = US_MicroSecond();
		usEchoReceived = 1;
	}
}
