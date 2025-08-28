/**
 * @file calibrate.h
 * @author Carole MEYER
 * @date 19 nov 2021
 * @version V1.1
 * @brief Calibration functions for the car application.
 * This file contains functions to perform calibration of the steering module.
 *
 * Version 1.0: Initial release
 * Version 1.1: 08/2025 - Rework of the steering calibration function (DI MERCURIO Sebastien)
 */

#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
*	Performe la calibration du module de direction (action moteur, recuperation valeur capteur)
**/
void CAL_SteeringCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATE_H__ */
