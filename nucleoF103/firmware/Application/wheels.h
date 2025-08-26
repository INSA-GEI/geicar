#ifndef _WHEELS_H_
#define _WHEELS_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define MAX_SPEED_FORWARD 75
#define MAX_SPEED_BACKWARD 25

extern uint32_t VMG_mes;
extern uint32_t VMD_mes;
extern uint32_t per_vitesseG;
extern uint32_t per_vitesseD;

extern int nbImpulsionG;
extern int nbImpulsionD;

void WHEELS_Init();

void WHEELS_OverflowManager();
/**
* Set the speed of the left and right rear motors. Speed values have to be between 0% and 100%
**/
void WHEELS_SetSpeed(GPIO_PinState en_right, GPIO_PinState en_left, int speed_right, int speed_left);

#define WHEELS_MOTOR_LEFT  0
#define WHEELS_MOTOR_RIGHT 1
uint32_t WHEELS_GetSensor(uint8_t motor);
uint32_t WHEELS_GetPERVitesse(uint8_t motor);
int WHEELS_GetOdometer(uint8_t motor);

#endif
