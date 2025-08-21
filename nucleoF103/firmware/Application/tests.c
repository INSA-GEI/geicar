/*
 * tests.c
 *
 *  Created on: Aug 21, 2025
 *      Author: dimercur
 */

#include "tests.h"
#include "steering.h"
#include "wheels.h"
#include "ultrasound.h"
#include "can_communication.h"
#include "measures.h"
#include "control.h"
#include "calibrate.h"
#include "power.h"

#include <stdio.h>

#define CAN_ID_TEST 0x123 // Define a test CAN ID for communication

void TESTS_Run(void) {
	// Run all tests
	printf("Running tests...\r\n");

	// Test steering
	printf("Testing steering...\r\n");

	STEERING_SetAngle(100); // Set to center
	HAL_Delay(1000);
	STEERING_SetAngle(0); // Set to left
	HAL_Delay(1000);
	STEERING_SetAngle(200); // Set to right
	HAL_Delay(1000);
	STEERING_SetAngle(100); // Set back to center

	printf("Steering test completed.\r\n");

	// Test wheels
	printf("Testing wheels...\r\n");

	// left motor
	WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 50, 0); // Motor left stopped (50 = no movement), right stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 0, 0); // Motor left full backward (0 = back), right stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 50, 0); // Motor left stopped (50 = no movement), right stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 100, 0); // Motor left full forward (100 = front), right stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 50, 0); // Motor left stopped (50 = no movement), right stopped

	// right motor
	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0, 50); // Motor right stopped (50 = no movement), left stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0,  0); // Motor right full backward (0 = back), left stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0,  50); // Motor right stopped (50 = no movement), left stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0,  100); // Motor right full forward (100 = front), left stopped
	HAL_Delay(2000);
	WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0,  50); // Motor right stopped (50 = no movement), left stopped
	printf("Wheels test completed.\r\n");

	// Test ultrasonic sensors
	printf("Testing ultrasonic sensors...\r\n");
	//US_Init();
	for (int i = 0; i < 6; i++) {
		uint32_t distance = US_GetDistance(i);
		printf("US[%d] Distance: %lu cm\r\n", i, distance);
	}

	printf("Ultrasonic sensors test completed.\r\n");

	// Test CAN communication
	printf("Testing CAN communication...\r\n");
	//CAN_COM_Init();
	CAN_COM_Send(CAN_ID_TEST, (uint8_t*) "Test", 4);

	printf("CAN communication test completed.\r\n");

	// Test measures
	printf("Testing measures...\r\n");
	MEASURES_Init();
	uint16_t batteryLevel = MEASURES_GetBatteryLevel();
	uint16_t steeringAngle = MEASURES_GetSteeringAngle();
	uint16_t motorLeftCurrent = MEASURES_GetMotorLeftCurrent();
	uint16_t motorRightCurrent = MEASURES_GetMotorRightCurrent();

	printf("Battery Level: %d\r\n", batteryLevel);
	printf("Steering Angle: %d\r\n", steeringAngle);
	printf("Left Motor Current: %d\r\n", motorLeftCurrent);
	printf("Right Motor Current: %d\r\n", motorRightCurrent);

	printf("Measures test completed.\r\n");

	// Test control logic
	printf("Testing control logic...\r\n");

	printf("Tests finished successfully.\r\n");
	for(;;);
}

