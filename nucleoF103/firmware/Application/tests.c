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

#define TEST_STEERING 	0
#define TEST_WHEELS   	0
#define TEST_ULTRASONIC 0
#define TEST_CAN      	0
#define TEST_MEASURES 	0
#define TEST_CONTROL  	0
#define TEST_CALIBRATE 	0
#define TEST_WHEELS_SENSORS 0

void TESTS_Run(void) {
	// Run all tests
	printf("Running tests...\r\n");

	while (1) {
#if (TEST_STEERING == 1)

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
#endif /* TEST_STEERING */

#if (TEST_WHEELS == 1)
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

#endif /* TEST_WHEELS */

		//WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_RESET, 100, 0); // Motor left stopped (50 = no movement), right stopped
		//WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_SET, 0,  100); // Motor right full forward (100 = front), left stopped
#if (TEST_WHEELS_SENSORS == 1)
		// Test wheels sensors
		printf("Testing wheels sensors...\r\n");

		WHEELS_SetSpeed(GPIO_PIN_SET, GPIO_PIN_SET, 100, 100); // Motor left forward (100 = front), right full forward (100 = front)

		for (int i = 0; i < 30; i++) {
            HAL_Delay(1000);
            printf("Left wheel sensor: %lu\r\n", WHEELS_GetSensor(WHEELS_MOTOR_LEFT));
            printf("Left wheel PER: %lu\r\n", WHEELS_GetPERVitesse(WHEELS_MOTOR_LEFT));
            printf("Left wheel Odo: %i\r\n\n", WHEELS_GetOdometer(WHEELS_MOTOR_LEFT));

            printf("Right wheel sensor: %lu\r\n", WHEELS_GetSensor(WHEELS_MOTOR_RIGHT));
            printf("Right wheel PER: %lu\r\n", WHEELS_GetPERVitesse(WHEELS_MOTOR_RIGHT));
            printf("Right wheel Odo: %i\r\n\n\n\n", WHEELS_GetOdometer(WHEELS_MOTOR_RIGHT));
        }

		WHEELS_SetSpeed(GPIO_PIN_RESET, GPIO_PIN_RESET, 50, 50); // Motor left stop, right stop

		printf("Wheels sensors test completed.\r\n");
#endif /* TEST_WHEELS_SENSORS */

#if (TEST_ULTRASONIC == 1)
		// Test ultrasonic sensors
		printf("Testing ultrasonic sensors...\r\n");
		for (int i = 0; i < 6; i++) {
			uint32_t distance = US_GetDistance(i);
			printf("US[%d] Distance: %lu cm\r\n", i, distance);
		}

		printf("Ultrasonic sensors test completed.\r\n");

#endif /* TEST_ULTRASONIC */

#if (TEST_CAN == 1)
		// Test CAN communication
		printf("Testing CAN communication...\r\n");
		CAN_COM_Send(CAN_ID_TEST, (uint8_t*) "Test", 4);

		printf("CAN communication test completed.\r\n");
#endif /* TEST_CAN */

#if (TEST_MEASURES == 1)

		// Test measures
		printf("Testing measures...\r\n");
		uint16_t batteryLevel = MEASURES_GetBatteryLevel();
		uint16_t steeringAngle = MEASURES_GetSteeringAngle();
		uint16_t motorLeftCurrent = MEASURES_GetMotorLeftCurrent();
		uint16_t motorRightCurrent = MEASURES_GetMotorRightCurrent();

		printf("Battery Level: %d\r\n", batteryLevel);
		printf("Steering Angle: %d\r\n", steeringAngle);
		printf("Left Motor Current: %d\r\n", motorLeftCurrent);
		printf("Right Motor Current: %d\r\n", motorRightCurrent);

		printf("Measures test completed.\r\n\n\n");
#endif /* TEST_MEASURES */

#if (TEST_CALIBRATE == 1)
		// Test calibration
		printf("Testing calibration...\r\n");
		CALIBRATE_RunSteeringCalibration();
		printf("Steering calibration completed.\r\n");

		printf("Calibration test completed.\r\n");
#endif /* TEST_CALIBRATE */

#if (TEST_CONTROL == 1)
		// Test control logic
		printf("Testing control logic...\r\n");

		CONTROL_SetMode(CONTROL_MODE_AUTONOMOUS);
		CONTROL_SetTargetSpeed(60); // Set target speed to 60%
		CONTROL_SetTargetSteeringAngle(150); // Set target steering angle to 150 (right)
		HAL_Delay(5000); // Let it run for 5 seconds
		CONTROL_SetMode(CONTROL_MODE_MANUAL);
		CONTROL_SetTargetSpeed(0); // Stop the vehicle
		CONTROL_SetTargetSteeringAngle(100); // Center the steering
		printf("Control logic test completed.\r\n");
#endif /* TEST_CONTROL */

		HAL_Delay(2000); // Wait 5 seconds before repeating the tests
	}
	for(;;);
}

