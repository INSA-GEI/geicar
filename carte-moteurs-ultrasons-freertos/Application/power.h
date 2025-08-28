/**
 * @file power.c
 * @author Pehladik
 * @version V1.0
 * @date 26 Octobre 2019
 * @brief Power management for the car application.
 * This file contains functions to manage the power state of the car, including bootstrapping and shutdown procedures.
 *
 * - Version 1.0 : Initial release
 */

#ifndef __POWER_H__
#define __POWER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
* Set the power
**/
void POWER_Boostrap(void);

/**
* shutdown the power
**/
void POWER_Shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_H__ */
