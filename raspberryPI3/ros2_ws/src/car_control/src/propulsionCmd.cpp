#include "../include/car_control/propulsionCmd.h"



/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd){

    if (reverse){
        leftRearPwmCmd = 50 - 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 0] (reverse)

    } else{
        leftRearPwmCmd = 50 + 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 100] (forward)
    }

    rightRearPwmCmd = leftRearPwmCmd;

    return 0;

}