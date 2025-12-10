#include "../include/car_control/steeringCmd.h"



//return the Pwm command to reach the angle passed in argument
// /!\ NOT USED NOW /!\ currentSteerAngle is directly sent to steering motor in car_control_node.cpp
int steeringCmd(float requestedSteerAngle, float currentSteerAngle, uint8_t & steeringPwmCmd){

	float errorAngle = currentSteerAngle - requestedSteerAngle;

    //Command's calculation
	if (abs(errorAngle)<TOLERANCE_ANGLE){
		steeringPwmCmd = STOP;
	}
	else {
		if (errorAngle>0) {
			steeringPwmCmd = MAX_PWM_LEFT;
		}
		else {
			steeringPwmCmd = MAX_PWM_RIGHT;
		}
	}

    return errorAngle;
}