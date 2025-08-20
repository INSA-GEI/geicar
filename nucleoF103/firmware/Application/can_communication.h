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

#define CAN_ID_MOTORS_DATA	0x200	// Odometry, Motors speed, Steering angle
#define CAN_ID_MOTORS_CMD 	0x100	// Speed and Steering Commands
#define CAN_ID_US1	0x211			//Front US [cm]
#define CAN_ID_US2	0x221			//Rear US [cm]
#define CAN_ID_BATT_LEVEL	0x273	//Battery Level
#define CAN_ID_CALIBRATION_MODE	0x400
#define CAN_ID_COMM_CHECKING 0x410

#define CALIBRATION_REQUEST	0x1		//frame[0]
#define CALIBRATION_IN_PROGRESS 0x2	//frame[0]
#define CALIBRATION_SUCCESS 0x3		//frame[0]
#define CALIBRATION_FAIL 0x4		//frame[0]

#define CALIBRATION_USER_NEED 0x1	//frame[1]

#define COMM_CHECKING_REQUEST 0x1 //frame[0]
#define COMM_CHECKING_ACK 0x1	  //frame[1]

void CAN_COM_Init(void);
void CAN_COM_FilterConfig(void);
void CAN_COM_Send(uint32_t id, uint8_t* data, uint8_t length) ;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_COMMUNICATION_H__ */

