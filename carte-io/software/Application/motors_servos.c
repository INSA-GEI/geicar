/*
 * motors_servos.c
 *
 *  Created on: May 21, 2025
 *      Author: dimercur
 */

#include "motors_servos.h"
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "messages.h"

#include <stdint.h>
#include <math.h> // Pour la fonction roundf

extern void MX_TIM5_Init(void);
extern void MX_TIM8_Init(void);

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

HAL_StatusTypeDef MOTORS_SERVOS_SetMotorSpeed(uint8_t motor_id, uint16_t speed);
HAL_StatusTypeDef MOTORS_SERVOS_SetServoPosition(uint8_t servo_id, uint16_t position);
HAL_StatusTypeDef MOTORS_SERVOS_ConfigurePPM(uint8_t motor_id, uint16_t min_value, uint16_t max_value, uint16_t neutral_value);
uint32_t MOTORS_SERVOS_convert_us01_to_timer_ticks(uint16_t duration_us01);
HAL_StatusTypeDef MOTORS_SERVOS_configure_servo_pwm(uint32_t channel,
										uint16_t min_duration_us01,
                                     uint16_t max_duration_us01,
                                     uint16_t center_duration_us01,
                                     int16_t percentage_us01, // int16_t car peut être négatif
                                     uint32_t timer_arr_value);
HAL_StatusTypeDef MOTORS_SERVOS_configure_motors_pwm(uint32_t channel,
										uint16_t min_duration_us01,
                                      uint16_t max_duration_us01,
                                      uint16_t center_duration_us01,
                                      int16_t percentage_us01,
                                      uint16_t timer_arr_value); // ARR pour TIM8 est uint16_t

typedef struct {
	uint16_t min_duration_us01; // Durée minimale de l'impulsion en 0.01 ms
	uint16_t max_duration_us01; // Durée maximale de l'impulsion en 0.01 ms
	uint16_t center_duration_us01; // Durée de l'impulsion au point central en 0.01 ms
} ServoConfig_TypeDef;

static ServoConfig_TypeDef MOTORS_SERVOS_ServoConfig[4] = {0}; // Configuration des servos (4 servos max)
static ServoConfig_TypeDef MOTORS_SERVOS_MotorConfig[4] = {0}; // Configuration des moteurs (4 moteurs max)

/**
 * @brief  Fonction d'initialisation des moteurs et servos
 * @retval None
 */
void MOTORS_SERVOS_Init(void) {
	printf ("[MOTORS SENSORS] Initialisation... ");

	/* Initialisation du timer TIM5 (moteurs) */
	MX_TIM5_Init();

	/* Initialisation du timer TIM8 (servos) */
	MX_TIM8_Init();

	printf ("Done\n");
}

/**
 * @brief  Fonction de traitement des messages pour les moteurs et servos
 * @param  msg: Pointeur vers le message à traiter
 * @retval HAL_StatusTypeDef: Statut de la fonction
 */
HAL_StatusTypeDef MOTORS_SERVOS_MessageProcessor(Messages_TypeDef *msg) {
	HAL_StatusTypeDef status = HAL_OK;

	switch (msg->id) {
	case MSG_ID_I2C_SENSORS_10MS_EVENT:
		break;
	case MSG_ID_I2C_SENSORS_50MS_EVENT:
		break;
	default:
		status = HAL_ERROR;
		break;
	}

	return status;

}

HAL_StatusTypeDef MOTORS_SERVOS_SetMotorSpeed(uint8_t motor_id, uint16_t speed) {
	HAL_StatusTypeDef status = HAL_OK;

	if (motor_id < 4) { // Vérifier que l'ID du moteur est valide
		status=MOTORS_SERVOS_configure_motors_pwm(motor_id,
				MOTORS_SERVOS_MotorConfig[motor_id].min_duration_us01,
				MOTORS_SERVOS_MotorConfig[motor_id].max_duration_us01,
				MOTORS_SERVOS_MotorConfig[motor_id].center_duration_us01, speed,
				htim5.Instance->ARR); // Passer la valeur de l'ARR du timer
	} else {
		status = HAL_ERROR;
	}
	return status;
}

HAL_StatusTypeDef MOTORS_SERVOS_SetServoPosition(uint8_t servo_id, uint16_t position){
	HAL_StatusTypeDef status = HAL_OK;

	if (servo_id < 4) { // Vérifier que l'ID du servo est valide
		status=MOTORS_SERVOS_configure_servo_pwm(servo_id,
				MOTORS_SERVOS_ServoConfig[servo_id].min_duration_us01,
				MOTORS_SERVOS_ServoConfig[servo_id].max_duration_us01,
				MOTORS_SERVOS_ServoConfig[servo_id].center_duration_us01,
				position, htim8.Instance->ARR); // Passer la valeur de l'ARR du timer
	} else {
		status = HAL_ERROR;
	}
	return status;
}

HAL_StatusTypeDef MOTORS_SERVOS_ConfigurePPM(uint8_t motor_id, uint16_t min_value, uint16_t max_value, uint16_t neutral_value) {
	HAL_StatusTypeDef status = HAL_OK;

	if (motor_id < 4) { // Vérifier que l'ID du moteur est valide
		MOTORS_SERVOS_MotorConfig[motor_id].min_duration_us01 = min_value;
		MOTORS_SERVOS_MotorConfig[motor_id].max_duration_us01 = max_value;
		MOTORS_SERVOS_MotorConfig[motor_id].center_duration_us01 = neutral_value;
	} else {
		status = HAL_ERROR;
	}

	return status;
}

// Définir la durée d'un tick du timer en ms pour la clarté (calculée ci-dessus)
// C'est la valeur inverse de la fréquence du compteur (120MHz / 234)
#define TIMER_TICK_DURATION_MS_FLOAT (1.0f / (120000000.0f / 234.0f) * 1000.0f) // Environ 0.00195 ms

/**
 * @brief Convertit une durée exprimée en 0.01 ms en "ticks" du timer.
 * Utilise la durée réelle d'un tick du timer pour la conversion.
 * @param duration_us01 Durée en unités de 0.01 ms (ex: 1500 pour 1.5ms)
 * @return Valeur de comparaison pour le registre CCRx (en ticks du timer)
 */
uint32_t MOTORS_SERVOS_convert_us01_to_timer_ticks(uint16_t duration_us01) {
    // Convertir la durée en 0.01ms en une durée totale en millisecondes
    float total_duration_ms = (float)duration_us01 * 0.01f;

    // Convertir la durée totale en millisecondes en nombre de ticks du timer
    // Un tick dure TIMER_TICK_DURATION_MS_FLOAT ms
    float ticks_float = total_duration_ms / TIMER_TICK_DURATION_MS_FLOAT;

    // Arrondir à l'entier le plus proche et retourner
    return (uint32_t)roundf(ticks_float);
}

/*
// Exemple de test de la conversion :
// Pour 1.5 ms (1500 unités de 0.01ms)
// total_duration_ms = 1500 * 0.01 = 15.0 ms
// ticks_float = 15.0 / 0.00195 = 7692.3 ticks
// roundf(7692.3) = 7692

// Pour 2.5 ms (2500 unités de 0.01ms)
// total_duration_ms = 2500 * 0.01 = 25.0 ms
// ticks_float = 25.0 / 0.00195 = 12820.5 ticks
// roundf(12820.5) = 12821

// Ah, petite correction ici: un servo s'attend à des durées d'impulsion *beaucoup plus courtes* que 15ms ou 25ms.
// Les valeurs typiques sont 0.5ms à 2.5ms.
// Si "duration_us01" est 1500, cela représente 1.5ms (1500 * 0.01ms = 1.5ms). C'est bien une durée d'impulsion de servo.
// Le calcul est donc correct pour l'unité.

// Reprenons l'exemple avec des durées de servo typiques:
// 1.5 ms (1500 en 0.01ms_unit)
// total_duration_ms = 1500 * 0.01 = 1.5 ms
// ticks_float = 1.5 / 0.00195 = 769.23 ticks
// roundf(769.23) = 769

// 0.5 ms (500 en 0.01ms_unit)
// total_duration_ms = 500 * 0.01 = 0.5 ms
// ticks_float = 0.5 / 0.00195 = 256.41 ticks
// roundf(256.41) = 256

// 2.5 ms (2500 en 0.01ms_unit)
// total_duration_ms = 2500 * 0.01 = 2.5 ms
// ticks_float = 2.5 / 0.00195 = 1282.05 ticks
// roundf(1282.05) = 1282

// Ces valeurs semblent cohérentes avec un ARR de 10240 (puisque la durée d'impulsion doit être inférieure à la période).
*/


// Supposons que le TIM5 a plusieurs canaux. Définissez le canal que vous utilisez.
// Par exemple, TIM_CHANNEL_1, TIM_CHANNEL_2, etc.
#define SERVO_TIMER_CHANNEL TIM_CHANNEL_1 // Adaptez ceci à votre configuration

/**
 * @brief Configure la sortie PWM pour un servomoteur en fonction d'une position en pourcentage.
 * @param channel Le canal du timer à utiliser (ex: TIM_CHANNEL_1)
 * @param min_duration_us01 Durée minimale de l'impulsion en 0.01 ms (ex: 500 pour 0.5ms)
 * @param max_duration_us01 Durée maximale de l'impulsion en 0.01 ms (ex: 2500 pour 2.5ms)
 * @param center_duration_us01 Durée de l'impulsion au point central en 0.01 ms (ex: 1500 pour 1.5ms)
 * @param percentage_us01 Pourcentage de position du servo (-100.00 à +100.00) exprimé en 0.01%
 * Ex: 0 pour centre, 10000 pour +100%, -10000 pour -100%
 * @param timer_arr_value La valeur de l'Auto-Reload Register (ARR) du timer (ex: 10240)
 * Passer cette valeur pour garantir la cohérence des calculs.
 * @return HAL_StatusTypeDef indiquant le succès ou l'échec de l'opération (HAL_OK si OK)
 */
HAL_StatusTypeDef MOTORS_SERVOS_configure_servo_pwm(uint32_t channel,
									uint16_t min_duration_us01,
                                     uint16_t max_duration_us01,
                                     uint16_t center_duration_us01,
                                     int16_t percentage_us01, // int16_t car peut être négatif
                                     uint32_t timer_arr_value)
{
    // Convertir les durées min/max/centre en ticks du timer
    uint32_t min_ticks = MOTORS_SERVOS_convert_us01_to_timer_ticks(min_duration_us01);
    uint32_t max_ticks = MOTORS_SERVOS_convert_us01_to_timer_ticks(max_duration_us01);
    uint32_t center_ticks = MOTORS_SERVOS_convert_us01_to_timer_ticks(center_duration_us01);

    // Calcul de la plage de mouvement pour le servomoteur
    // La plage est asymétrique autour du centre si (max_ticks - center_ticks) != (center_ticks - min_ticks)
    float positive_range_ticks = (float)(max_ticks - center_ticks);
    float negative_range_ticks = (float)(center_ticks - min_ticks);

    // Calcul de la valeur de comparaison (CCR) pour le timer
    float target_ticks_float;

    // Convertir le pourcentage en float (-1.00 à +1.00)
    float normalized_percentage = (float)percentage_us01 / 10000.0f; // Ex: 5000 (50%) -> 0.5f

    if (normalized_percentage >= 0.0f) {
        // Mouvement vers la position maximale (+100%)
        target_ticks_float = (float)center_ticks + (positive_range_ticks * normalized_percentage);
    } else {
        // Mouvement vers la position minimale (-100%)
        // Le pourcentage est négatif, donc on ajoute un nombre négatif (ou soustrait un positif)
        target_ticks_float = (float)center_ticks + (negative_range_ticks * normalized_percentage);
    }

    // Assurez-vous que la valeur reste dans les limites min_ticks et max_ticks
    if (target_ticks_float < (float)min_ticks) {
        target_ticks_float = (float)min_ticks;
    } else if (target_ticks_float > (float)max_ticks) {
        target_ticks_float = (float)max_ticks;
    }
    // Une vérification supplémentaire pour s'assurer que la valeur ne dépasse pas ARR+1
    // (bien que les min/max ticks devraient déjà être dans cette plage)
    if (target_ticks_float > (float)timer_arr_value + 1) { // ARR est la valeur max du compteur, donc ARR+1 est le nombre de ticks
        target_ticks_float = (float)timer_arr_value + 1;
    }


    // Arrondir à l'entier le plus proche pour le registre CCR
    uint32_t final_ccr_value = (uint32_t)roundf(target_ticks_float);

    // Appliquer la valeur au registre de comparaison du timer
    // En utilisant la bibliothèque HAL
    __HAL_TIM_SET_COMPARE(&htim5, channel, final_ccr_value);

    return HAL_OK;
}

// Définissez le canal que vous utilisez pour le moteur (ex: TIM_CHANNEL_1, TIM_CHANNEL_2, etc.)
// Si vous contrôlez plusieurs moteurs, vous aurez plusieurs appels avec des canaux différents.
#define MOTOR_TIMER_CHANNEL TIM_CHANNEL_1 // Adaptez ceci à votre configuration

/**
 * @brief Configure la sortie PWM pour un moteur ou servomoteur en fonction d'une position/vitesse en pourcentage.
 * Prévue pour utiliser le TIM8 (16 bits).
 * @param channel Le canal PWM spécifique de TIM8 à configurer (ex: TIM_CHANNEL_1)
 * @param min_duration_us01 Durée minimale de l'impulsion en 0.01 ms (ex: 500 pour 0.5ms)
 * @param max_duration_us01 Durée maximale de l'impulsion en 0.01 ms (ex: 2500 pour 2.5ms)
 * @param center_duration_us01 Durée de l'impulsion au point central en 0.01 ms (ex: 1500 pour 1.5ms)
 * @param percentage_us01 Pourcentage de position/vitesse du moteur (-100.00 à +100.00) exprimé en 0.01%
 * Ex: 0 pour centre, 10000 pour +100%, -10000 pour -100%
 * @param timer_arr_value La valeur de l'Auto-Reload Register (ARR) du timer (ex: 10239 pour une période de 10240 ticks)

 * @return HAL_StatusTypeDef indiquant le succès ou l'échec de l'opération (HAL_OK si OK)
 */
HAL_StatusTypeDef MOTORS_SERVOS_configure_motors_pwm(uint32_t channel,
										uint16_t min_duration_us01,
                                      uint16_t max_duration_us01,
                                      uint16_t center_duration_us01,
                                      int16_t percentage_us01,
                                      uint16_t timer_arr_value) // ARR pour TIM8 est uint16_t
{
    // Convertir les durées min/max/centre en ticks du timer (uint32_t pour les calculs intermédiaires)
    uint32_t min_ticks_calc = MOTORS_SERVOS_convert_us01_to_timer_ticks(min_duration_us01);
    uint32_t max_ticks_calc = MOTORS_SERVOS_convert_us01_to_timer_ticks(max_duration_us01);
    uint32_t center_ticks_calc = MOTORS_SERVOS_convert_us01_to_timer_ticks(center_duration_us01);

    // Assurez-vous que les valeurs calculées ne dépassent pas la capacité 16 bits de TIM8
    // Le maximum pour un uint16_t est 65535. Votre ARR est 10239, donc c'est bon.
    if (min_ticks_calc > timer_arr_value + 1 ||
        max_ticks_calc > timer_arr_value + 1 ||
        center_ticks_calc > timer_arr_value + 1)
    {
        // Gérer l'erreur si les durées sont trop grandes pour le timer 16 bits
        // ou si elles dépassent la période.
        // Par exemple, vous pouvez imprimer une erreur ou limiter les valeurs.
        // Pour l'instant, on se base sur le fait que vos valeurs 0.5ms-2.5ms sont bien en dessous de 10240 ticks.
        return HAL_ERROR; // Ou un code d'erreur personnalisé
    }

    // Convertir les valeurs calculées en uint16_t pour les opérations suivantes si nécessaire
    uint16_t min_ticks = (uint16_t)min_ticks_calc;
    uint16_t max_ticks = (uint16_t)max_ticks_calc;
    uint16_t center_ticks = (uint16_t)center_ticks_calc;


    // Calcul de la plage de mouvement
    float positive_range_ticks = (float)(max_ticks - center_ticks);
    float negative_range_ticks = (float)(center_ticks - min_ticks);

    // Calcul de la valeur de comparaison (CCR) pour le timer
    float target_ticks_float;

    // Convertir le pourcentage en float (-1.00 à +1.00)
    float normalized_percentage = (float)percentage_us01 / 10000.0f;

    if (normalized_percentage >= 0.0f) {
        target_ticks_float = (float)center_ticks + (positive_range_ticks * normalized_percentage);
    } else {
        target_ticks_float = (float)center_ticks + (negative_range_ticks * normalized_percentage);
    }

    // Assurez-vous que la valeur reste dans les limites min_ticks et max_ticks
    if (target_ticks_float < (float)min_ticks) {
        target_ticks_float = (float)min_ticks;
    } else if (target_ticks_float > (float)max_ticks) {
        target_ticks_float = (float)max_ticks;
    }

    // S'assurer que la valeur ne dépasse pas le maximum possible pour le ARR du timer
    // (ARR + 1 représente le nombre total de ticks)
    if (target_ticks_float > (float)(timer_arr_value + 1)) {
        target_ticks_float = (float)(timer_arr_value + 1);
    }

    // Arrondir à l'entier le plus proche et le caster en uint16_t pour le registre CCR
    uint16_t final_ccr_value = (uint16_t)roundf(target_ticks_float);

    // Appliquer la valeur au registre de comparaison du timer
    // En utilisant la bibliothèque HAL pour TIM8
    __HAL_TIM_SET_COMPARE(&htim8, channel, final_ccr_value);

    return HAL_OK;
}
