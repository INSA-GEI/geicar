/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_COMMUNICATION_H__
#define __CAN_COMMUNICATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app.h"

#define CAN_ID_MOTORS_DATA		0x200	// Odometry, Motors speed => to Raspberry
#define CAN_ID_US1				0x211	// Front Ultrasonic mesures [cm] => to Raspberry
#define CAN_ID_US2				0x221	// Rear Ultrasonic mesures [cm] => to Raspberry
#define CAN_ID_BATT_LEVEL		0x273	// Battery Level => to Raspberry
#define CAN_ID_MOTORS_CMD 		0x100	// Motors speed and steering angle <= from Raspberry
#define CAN_ID_CALIBRATION_MODE	0x400   // Calibration mode <= from Raspberry and => to Raspberry
#define CAN_ID_COMM_CHECKING 	0x410   // Communication checking <= from Raspberry and => to Raspberry

// Define for calibration mode
#define CALIBRATION_REQUEST		0x1		// frame[0]	cmd for entering calibration mode (<= from Raspberry)
#define CALIBRATION_IN_PROGRESS 0x2		// frame[0] info indicating calibration is in progress (=> to Raspberry)
#define CALIBRATION_SUCCESS 	0x3		// frame[0] status indicating calibration was successful (=> to Raspberry)
#define CALIBRATION_FAIL 		0x4		// frame[0] status indicating calibration failed (=> to Raspberry)

#define CALIBRATION_USER_NEED 	0x1		// frame[1]

#define COMM_CHECKING_REQUEST 	0x1 	// frame[0] cmd for communication checking (<= from Raspberry)
#define COMM_CHECKING_ACK 		0x1	  	// frame[1] ack for communication checking (=> to Raspberry)

typedef struct {
	AppMessage_typeDef header;
	uint16_t can_id; 	// ID of received CAN frame
	uint8_t length;		// Length of received CAN frame
	uint8_t data[8];	// Data of received CAN frame
} CANReceivedFrame_typeDef;

void CAN_COM_Init(void);
void CAN_COM_Send(uint32_t id, uint8_t* data, uint8_t length) ;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_COMMUNICATION_H__ */

