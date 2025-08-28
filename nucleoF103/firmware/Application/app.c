/**
 * @file app.c
 * @author Sebastien DI MERCURIO
 * @version V1.0
 * @date 20 Aout 2023
 *
 * @brief Main application file.
 * This file contains the main application logic, including initialization and the main loop.
 * It handles the control of the car's motors, ultrasonic sensors, and communication via CAN.
 */

#include "app.h"

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "power.h"
#include "FLASH_PAGE_F1.h"
#include "steering.h"
#include "wheels.h"
#include "ultrasound.h"
#include "control.h"
#include "calibrate.h"
#include "can_communication.h"
#include "measures.h"

#include <stdio.h>

#if defined (__TESTS__)
#include "tests.h"
#endif

/* Modes
 * 0- Calibration
 * 1- Control
 */
int mode = 1;

int UPDATE_CMD_FLAG = 1;
int US_FLAG = 1;
int CAN_SEND_MOTORS = 1;
int CAN_SEND_US = 0;
int CAN_SEND_BATT = 1;

/***************************
 * Ultrasonic sensors data *
 ***************************/
/* Current ultrasonic sensor index
 * 0- Front Left
 * 1- Front Center
 * 2- Front Right
 * 3- Rear Left
 * 4- Rear Center
 * 5- Rear Right
 */
int currentUs = 0;

/* usDistance[] : Ultrasonic measurements [cm]
 * usDistance[0] front left
 * usDistance[1] front center
 * usDistance[2] front right
 * usDistance[3] rear left
 * usDistance[4] rear center
 * usDistance[5] rear right
 */
uint16_t usDistance[6] = {0,0,0,0,0,0};

uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

// Speed cmd
int leftRearSpeed = -1;
int rightRearSpeed = -1;
int steeringAngle = -1;

// Periodic counters
static int cmpt_can_motors = 0;
static int cmpt_us = 0;
static int cmpt_batt = 0;

//Communication checking request
int commCheckingRequest = 0;

/**
 * @brief Initialize the application.
 * This function sets up the necessary software components
 */
void APP_Init(void){
	/* Initialisations */
	// Moteurs
	WHEELS_Init();

	// Steering
	STEERING_Init();

	// Analog measures (ADC1)
	MEASURES_Init();

	// Ultrasonic sensors
	US_Init();

	// CAN communications
	CAN_COM_Init();

    printf("FW Geicar motor - steering - ultrasonic sensors.\r\n");
	printf("Application version: %s\r\n\n", APP_VERSION);
}

/**
 * @brief Run the application.
 *
 * This function is the main loop of the application. It handles the main logic,
 * processes inputs, and updates outputs.
 *
 * @remark: this function never returns, it runs indefinitely.
 */
void APP_Run(void){
	printf("Application started\r\n");

#if defined (__TESTS__)
    TESTS_Run(); // Run tests if defined
#else

	/* Steering Initialization*/
	// Write default calibration values in flash memory (first use only)
	if ((int)Flash_Read_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_A_DEFAULT
			|| (int)Flash_Read_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_B_DEFAULT)
	{
		Flash_Write_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR, STEERING_CALIBRATION_A_DEFAULT);
		Flash_Write_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR, STEERING_CALIBRATION_B_DEFAULT);

		Flash_Write_NUM(STEERING_CALIBRATION_A_ADDR, STEERING_CALIBRATION_A_DEFAULT);
		Flash_Write_NUM(STEERING_CALIBRATION_B_ADDR, STEERING_CALIBRATION_B_DEFAULT);
	}

	while (1) {

		// Perform ultrasonic sensors measurements
		if (US_FLAG==1) {
			US_FLAG=0;

			// Manage ultrasonic sensors index and restart if needed
			if (currentUs >= 6)
				currentUs = 0;

			usDistance[currentUs]=US_GetDistance(currentUs);

			currentUs+=1;

			// When all the us sensors have been performed,
			// send data to can
			if (currentUs == 6)
				CAN_SEND_US = 1;
		}

		// Update motors commands
		if (UPDATE_CMD_FLAG) {
			UPDATE_CMD_FLAG = 0;

			if (mode == 0) {	//Calibration Mode
				CAL_SteeringCalibration();
				mode = 1;
			} else {	//Control Mode
				CAR_CONTROL_Manage(leftRearSpeed,rightRearSpeed, steeringAngle);
			}
		}

		/* CAN : Sending motors data */
		if (CAN_SEND_MOTORS) {

			//Number of sensor pulses since last message (left rear wheel and right rear wheel)
			data[0] = nbImpulsionG;
			data[1] = nbImpulsionD;
			nbImpulsionG = 0;
			nbImpulsionD = 0;

			data[2] = (uint8_t)((VMG_mes >> 8) & 0xFF); // Left Rear Speed MSB
			data[3] = (uint8_t)(VMG_mes & 0xFF); 	//LSB

			data[4] = (uint8_t)((VMD_mes >> 8) & 0xFF); // Right Rear Speed MSB
			data[5] = (uint8_t)(VMD_mes & 0xFF); // LSB

			//data[6] = (uint8_t)(STEERING_GetAngle());	//Steering Angle MSB

			CAN_COM_Send(CAN_ID_MOTORS_DATA, data, 6);

			CAN_SEND_MOTORS = 0;
		}

		/* CAN : Sending battery data */
		if (CAN_SEND_BATT) {
			//Battery Level

			uint16_t vbat = MEASURES_GetBatteryLevel();

			data[0] = (vbat >> 8) & 0xFF; // Vbat MSB
			data[1] = vbat & 0xFF; 	//LSB

			CAN_COM_Send(CAN_ID_BATT_LEVEL, data, 2);

			CAN_SEND_BATT= 0;
		}

		/* CAN : Sending ultrasonic sensors data */
		if (CAN_SEND_US) {
			//Sending US1 data (front)
			data[0] = (usDistance[0] >> 8) & 0xFF;	//US Front Left
			data[1] = usDistance[0] & 0xFF;

			data[2] = (usDistance[1] >> 8) & 0xFF;	//US Front Center
			data[3] = usDistance[1] & 0xFF;

			data[4] = (usDistance[2] >> 8) & 0xFF;	//US Front Right
			data[5] = usDistance[2] & 0xFF;

			CAN_COM_Send(CAN_ID_US1, data, 6);

			//Sending US2 data (rear)
			data[0] = (usDistance[3] >> 8) & 0xFF;	//US Rear Left
			data[1] = usDistance[3] & 0xFF;

			data[2] = (usDistance[4] >> 8) & 0xFF;	//US Rear Center
			data[3] = usDistance[4] & 0xFF;

			data[4] = (usDistance[5] >> 8) & 0xFF;	//US Rear Right
			data[5] = usDistance[5] & 0xFF;

			CAN_COM_Send(CAN_ID_US2, data, 6);

			CAN_SEND_US = 0;
		}

		/* CAN : communication checking */
		if (commCheckingRequest) {
			data[1] = COMM_CHECKING_ACK;

			CAN_COM_Send(CAN_ID_COMM_CHECKING, data, 1); //Send ack

			commCheckingRequest = 0;
		}
	}
#endif /* __TESTS__ */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2)	{
		WHEELS_OverflowManager();
	} else if (htim->Instance == TIM3) {
		US_OverflowManager();
	}
}

void APP_PeriodicCountersUpdate(void) {
	cmpt_can_motors ++;
	cmpt_us++;
	cmpt_batt++;

	if (cmpt_can_motors == PERIOD_SEND_MOTORS){
		CAN_SEND_MOTORS = 1;
		cmpt_can_motors = 0;
	}
	if (cmpt_us == PERIOD_UPDATE_US){
		US_FLAG = 1;
		cmpt_us = 0;
	}
	if (cmpt_batt == PERIOD_SEND_BATT){
		CAN_SEND_BATT = 1;
		cmpt_batt = 0;
	}
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  ch: Character to be printed
 * @retval Character sent
 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
