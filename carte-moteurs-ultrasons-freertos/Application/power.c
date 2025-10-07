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

#include "power.h"

#include "app.h"
#include "configuration.h"

/**
 * @brief Bootstrap the power system.
 * This function enables the power supply to the car's systems.
 * It sets the appropriate GPIO pin to enable power.
 */
void POWER_Boostrap(void) {
    /* auto-maintien alim */
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_SET);
}

/**
 * @brief Shutdown the power system.
 * This function disables the power supply to the car's systems.
 * It resets the appropriate GPIO pin to cut off power.
 */
void POWER_Shutdown(void) {
    //coupure alim
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER_ENABLE_GPIO_Port, POWER_ENABLE_Pin, GPIO_PIN_RESET);
}
