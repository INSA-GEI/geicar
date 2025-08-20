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
#include "tim.h"
#include "ultrasound.h"

// Variables pour les mesures US
int usEchoStart = 1;
int usEchoReceived = 0;
uint64_t usEchoRisingTime = 0;
uint64_t usEchoDuration=0;

uint16_t usTriggerPin[6] = {US_Front_Left_Trig_Pin,US_Front_Center_Trig_Pin,US_Front_Right_Trig_Pin,US_Rear_Left_Trig_Pin,US_Rear_Center_Trig_Pin,US_Rear_Right_Trig_Pin};

uint64_t timerOverflow = 0;
uint64_t microSecondTime = 0;

//Start TIM3 counter
void startMicroSecondCounter(){
	TIM3->CR1 |= TIM_CR1_CEN;
}



//return current time in microsecond
uint64_t microSecond(){
	microSecondTime = timerOverflow * TIM3->ARR + TIM3->CNT;
	return microSecondTime;

}

// EXTI External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(usEchoStart){ //check pin state : rising
		usEchoRisingTime = microSecond();
		usEchoStart = 0;

	}else{
		uint64_t echoDuration = microSecond() - usEchoRisingTime;

		if (echoDuration >= 0)
			usEchoDuration = echoDuration;
		else
			usEchoDuration=-1;

		usEchoStart = 1;
		usEchoReceived = 1;

	}
}

void US_MicroDelay(uint32_t delay)
{
	volatile uint32_t cnt=(delay*6)+5;

	while (cnt >0) {
		cnt--;
	}
}

void US_Init(void) {
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	startMicroSecondCounter();
}

//Count overflows
void US_OverflowManager(){
	timerOverflow +=1 ;
}

uint16_t US_GetDistance(uint8_t channel) {
	uint16_t distance=0;

	usEchoStart = 1;

	HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[channel], GPIO_PIN_SET); //Trigger ON
	US_MicroDelay(10);
	HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[channel], GPIO_PIN_RESET); //Trigger OFF

	HAL_Delay(40);	//Waiting to receive the echo

	if (usEchoReceived)	//If we received the echo
		distance = usEchoDuration/58;
	else //If the echo is not received (i.e. sensor failure)
		distance = 1000;	//Set distance value out of range

	usEchoReceived = 0;

	return distance;
}

