//
//  power.c
//  
//
//  Created by pehladik on 26/10/2019.
//

#include "power.h"

void POWER_Boostrap(void){
    /* auto-maintien alim */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
}


void POWER_Shutdown(void){
    //coupure alim
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}
