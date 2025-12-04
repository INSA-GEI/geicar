#ifndef __car_control_node_H
#define __car_control_node_H

#include <stdint.h>
#include <string.h>  


#define PERIOD_UPDATE_CMD 2ms //Period to update proupulsion and steering command in [ms]

#define STOP 50 //PWM value to stop motors

#define SOURCE_JOYSTICK 0
#define SOURCE_HMI      1

#define MODE_MANUAL         0
#define MODE_AUTONOMOUS     1
#define MODE_CALIBRATION    2

#endif /*__ car_control_node_H */