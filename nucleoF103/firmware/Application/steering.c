/**
 * @file steering.c
 * @brief Steering control functions
 * @date 2025
 * @author DI MERCURIO Sebastien
 * This file contains functions to control the steering motor, read the steering angle,
 * and manage button inputs for steering control.
 *
 * Modification 08/2025: Passage de la direction sur le timer 3 CH1 pour commander un servomoteur
 * à 100 Hz.
 * Valeur de configuration du timer 3 CH1:
 * CLk input: 62 MHz (APB1 Timer clock)
 * TIM3->PSC = 9; // Prescaler to get 6.2 MHz clock
 * TIM3->ARR = 63999; // Auto-reload value for 100 Hz
 *
 */
/* Includes ------------------------------------------------------------------*/

#include "steering.h"
#include "tim.h"
#include "gpio.h"
#include "measures.h"

int need_read_calibration = 1;

float steering_sensor_coef_a;
float steering_sensor_coef_b;

extern uint32_t ADCBUF[5];

/* Programs ------------------------------------------------------------------*/

void STEERING_Init(void) {
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Start PWM on TIM4 CH1 for steering motor
}

//Set the speed of the steering motor. Speed value has to be between 0% and 100%
void STEERING_SetSpeed(GPIO_PinState en_steering, int speed){

	//Normalize value between LEFT_MAX_SPEED_STEERING and RIGHT_MAX_SPEED_STEERING (because CAN order is between 0 and 100)
	speed = ((RIGHT_MAX_SPEED_STEERING-LEFT_MAX_SPEED_STEERING)/100.0) * speed + LEFT_MAX_SPEED_STEERING;

	/* Threshold rotating speed of steering wheels*/
	if (speed > RIGHT_MAX_SPEED_STEERING){
		speed = RIGHT_MAX_SPEED_STEERING;
	} else if (speed < LEFT_MAX_SPEED_STEERING){
		speed  = LEFT_MAX_SPEED_STEERING;
	}

	speed = 3200 * ( speed/ 100.0 );
	TIM1->CCR3 = speed;

	HAL_GPIO_WritePin( GPIOC, GPIO_PIN_12, en_steering);  //PC12  AV
}

//return the current angle between 0 (full left) and 200 (full right)
int STEERING_GetAngle(void){
	if (need_read_calibration==1){
		STEERING_ReadCalibrationData();
		need_read_calibration=0;
	}

	int steeringSensor = (int) ADCBUF[1];
	int currentAngle = (steering_sensor_coef_a*steeringSensor + steering_sensor_coef_b);
	if (currentAngle > ANGLE_RIGHT_VALUE){
		currentAngle = ANGLE_RIGHT_VALUE;

	}else if (currentAngle < ANGLE_LEFT_VALUE){
		currentAngle = ANGLE_LEFT_VALUE;
	}

	return currentAngle;
}

int STEERING_IsAButtonPressed(){
	return ((!HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel)) || (!HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel)));
}	

//move steering with L/R buttons
void STEERING_MoveWithButton(void){
	static int previous_value_right = GPIO_PIN_RESET;
	static int previous_value_left = GPIO_PIN_RESET;
	int current_value_right = !HAL_GPIO_ReadPin(GPIOB, pin_bt_right_front_wheel);
	int current_value_left = !HAL_GPIO_ReadPin(GPIOB, pin_bt_left_front_wheel);

	if (((current_value_right == GPIO_PIN_SET) && (current_value_left == GPIO_PIN_SET))
			|| ((current_value_right == GPIO_PIN_RESET) && (previous_value_right == GPIO_PIN_SET))
			|| ((current_value_left == GPIO_PIN_RESET) && (previous_value_left == GPIO_PIN_SET))
	){
		STEERING_SetSpeed(GPIO_PIN_RESET, NO_STEERING);
		previous_value_right = GPIO_PIN_RESET;
		previous_value_left = GPIO_PIN_RESET;
	} else if ((current_value_right == GPIO_PIN_SET) && (previous_value_right == GPIO_PIN_RESET)){
		STEERING_SetSpeed(GPIO_PIN_SET, RIGHT_MAX_SPEED_STEERING);
		previous_value_right = GPIO_PIN_SET;
	} else if ((current_value_left == GPIO_PIN_SET) && (previous_value_left == GPIO_PIN_RESET)){
		STEERING_SetSpeed(GPIO_PIN_SET, LEFT_MAX_SPEED_STEERING);
		previous_value_left = GPIO_PIN_SET;
	}
}

//Read steering coefficients (a and b) in flash memory
void STEERING_ReadCalibrationData(){
	steering_sensor_coef_a = Flash_Read_NUM(STEERING_CALIBRATION_A_ADDR);
	steering_sensor_coef_b = Flash_Read_NUM(STEERING_CALIBRATION_B_ADDR);
}
