/*
 * Application.c
 *
 *  Created on: Dec 5, 2025
 *      Author: dimercur
 */

#include "Application.h"
#include "main.h"

typedef uint8_t  bool;
#define true  1
#define false 0

// --- I2C/Registre ---
#define I2C_SLAVE_ADDRESS (0xAA) // Adresse 8-bits (0x55 << 1) ou 7-bits (0x55)
#define REG_CONTROL_STATUS 0x00
#define REG_DISTANCE_MM    0x01

// Registre 0x00 : Control & Status (Lecture/Écriture)
// [7] : Mesure Disponible (1=Oui, 0=Non) - Lecture seule
// [6] : Mode Continu (1=Activé, 0=Désactivé) - Lecture/Écriture
// [5] : Lancer Mesure Ponctuelle (1=Lancer, se remet à 0 automatiquement) - Écriture seule
// [4:0] : Réservé
volatile uint8_t control_status_reg = 0x00;
#define STATUS_MEASURE_READY_MASK   0x80
#define STATUS_CONTINUOUS_MODE_MASK 0x40
#define COMMAND_SINGLE_SHOT_MASK    0x20

// Registre 0x01 : Distance en mm (16-bits)
volatile uint16_t distance_mm_reg = 0xFFFF; // Initialisée à une valeur invalide

// --- SRF05 ---
#define TIMEOUT_US 30000 // Temps max d'attente de l'écho (~5m)
#define SOUND_SPEED_MM_PER_US 0.343f // Vitesse du son à 20°C (343 m/s)
#define PULSE_TRIGGER_US 10       // Durée du pulse de trigger (min 10us)

// Variables pour Input Capture
volatile uint32_t capture_value_1 = 0;
volatile uint32_t capture_value_2 = 0;
volatile bool     capture_done = false;

extern TIM_HandleTypeDef htim2; // Déclarer votre Timer Input Capture
extern I2C_HandleTypeDef hi2c1; // Déclarer votre I2C

/**
 * @brief Envoie l'impulsion de déclenchement (Trigger) au SRF05.
 * @param none
 * @return none
 */
void SRF05_Trigger_Pulse(void)
{
    // 1. Initialiser le Timer en mode Input Capture et armer les interruptions
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    // 2. Réinitialiser les variables de capture
    capture_done = false;
    control_status_reg &= ~STATUS_MEASURE_READY_MASK; // Distance non disponible

    // 3. Envoyer l'impulsion de Trigger (doit être >= 10 us)
    HAL_GPIO_WritePin(SRF05_TRIG_GPIO_Port, SRF05_TRIG_Pin, GPIO_PIN_SET);
    // Utiliser __HAL_TIM_GET_COUNTER() avec un Timer lent, ou __NOP() pour un petit délai
    // Ici, nous utilisons un délai basé sur le cœur pour simplifier l'exemple.
    // Idéalement, utilisez un Timer/Delay non bloquant pour le pulse.
    HAL_Delay(1); // Mettre 1ms pour assurer le déclenchement (dépassant largement 10us)

    HAL_GPIO_WritePin(SRF05_TRIG_GPIO_Port, SRF05_TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Calcule la distance en mm à partir de la durée mesurée.
 * @param duration_us Durée de l'impulsion d'écho en microsecondes.
 * @return Distance en millimètres (uint16_t).
 */
uint16_t SRF05_Calculate_Distance(uint32_t duration_us)
{
    // La durée mesurée est l'aller-retour. Il faut diviser par 2.
    // Distance = (Durée * Vitesse du son) / 2

    // Calcul en float pour la précision, puis arrondi à l'entier.
    float distance_float = (float)duration_us * SOUND_SPEED_MM_PER_US / 2.0f;

    if (distance_float > 65535.0f)
    {
        return 0xFFFF; // Overflow ou hors-portée
    }

    return (uint16_t)distance_float;
}

void APPLICATION_Init(void) {
	// Initialize application components here
	// À placer dans main.c, après MX_I2C1_Init()
	// Activer les interruptions pour la réception et la transmission I2C esclave
	HAL_I2C_EnableListen_IT(&hi2c1);
}

void APPLICATION_Run(void) {
	// Main application loop
	while (1) {
		// Le maître peut vouloir écrire dans le registre de contrôle (0x00) pour :
		    // 1. Lancer une mesure ponctuelle (bit 5)
		    // 2. Activer/Désactiver le mode continu (bit 6)

		    // Logique de Mesure Ponctuelle :
		    if (control_status_reg & COMMAND_SINGLE_SHOT_MASK)
		    {
		        SRF05_Trigger_Pulse();
		        // Le bit 5 est remis à zéro après le déclenchement
		        control_status_reg &= ~COMMAND_SINGLE_SHOT_MASK;
		    }

		    // Logique de Mesure Continue :
		    if (control_status_reg & STATUS_CONTINUOUS_MODE_MASK)
		    {
		        // Si la mesure précédente est terminée, lancer la suivante
		        if (capture_done)
		        {
		            SRF05_Trigger_Pulse();
		        }
		    }

		    // Note : Il faudrait idéalement utiliser un Timer ou un RTOS pour cadencer
		    // les mesures en mode continu (par exemple, une mesure toutes les 50ms).
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // Assurez-vous que c'est le bon Timer et le bon canal
    if (htim->Instance == TIM2)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            // Vérifier si c'est le premier ou le deuxième front
            if (capture_value_1 == 0)
            {
                // Premier front (montant) : Début de la mesure
                capture_value_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

                // Configurer pour capturer le front descendant (fin de l'écho)
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else if (!capture_done)
            {
                // Deuxième front (descendant) : Fin de la mesure
                capture_value_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

                // Arrêter l'Input Capture et réinitialiser à la polarité montante
                HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

                uint32_t duration_us; // Durée en ticks Timer

                // Calcul de la différence entre les captures (gestion de l'overflow/roll-over)
                if (capture_value_2 > capture_value_1)
                {
                    duration_us = capture_value_2 - capture_value_1;
                }
                else
                {
                    // Roll-over du compteur : (MAX_VALUE - C1) + C2 + 1
                    duration_us = (htim->Instance->ARR - capture_value_1) + capture_value_2 + 1;
                }

                // Convertir les ticks en microsecondes si nécessaire
                // (Dépend de la prédivision et du prescaler du Timer)
                // Pour simplifier, supposons que le Timer est configuré pour compter en microsecondes.
                // Si la fréquence du Timer est 1MHz, 1 tick = 1 us.

                // 4. Calculer la distance et mettre à jour le registre
                distance_mm_reg = SRF05_Calculate_Distance(duration_us);
                control_status_reg |= STATUS_MEASURE_READY_MASK;
                capture_value_1 = 0; // Réinitialiser pour la prochaine mesure
                capture_done = true;
            }
        }
    }
}

// --- I2C SLAVE TX (Maître lit, Esclave Transmet) ---
// Le maître demande une lecture du registre :
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Lecture de la transmission terminée (ACK envoyé par le maître)
    // Réarmer l'écoute
    HAL_I2C_EnableListen_IT(hi2c);
}

// L'esclave est prêt à transmettre le registre demandé :
uint8_t current_reg_address = 0xFF; // Adresse du registre en cours de lecture/écriture

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Réception du registre d'adresse terminée (écriture du maître)
    // La variable 'current_reg_address' contient maintenant l'adresse.

    // Réarmer l'écoute
    HAL_I2C_EnableListen_IT(hi2c);
}

// Le maître a envoyé un start et l'adresse de l'esclave (en R/W) :
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    uint8_t tx_buffer[2]; // Buffer pour l'envoi de la distance

    if (TransferDirection == I2C_DIRECTION_TX) // Le Maître veut lire (Esclave TX)
    {
        // Le maître a précédemment écrit l'adresse du registre à lire (current_reg_address)
        if (current_reg_address == REG_CONTROL_STATUS)
        {
            tx_buffer[0] = control_status_reg;
            HAL_I2C_Slave_Transmit_IT(hi2c, tx_buffer, 1);
        }
        else if (current_reg_address == REG_DISTANCE_MM)
        {
            // Les données 16-bit sont envoyées MSB puis LSB (Convention I2C/SPI)
            tx_buffer[0] = (uint8_t)(distance_mm_reg >> 8); // MSB
            tx_buffer[1] = (uint8_t)distance_mm_reg;        // LSB
            HAL_I2C_Slave_Transmit_IT(hi2c, tx_buffer, 2);
        }
        else
        {
            // Adresse de registre inconnue
            HAL_I2C_EnableListen_IT(hi2c);
        }
    }
    else // I2C_DIRECTION_RX (Le Maître veut écrire, Esclave RX)
    {
        // L'opération d'écriture I2C commence par l'adresse du registre, puis les données.
        // On prépare la réception du premier octet, qui sera l'adresse du registre.
        HAL_I2C_Slave_Receive_IT(hi2c, &current_reg_address, 1);
    }
}

// Gestion des erreurs I2C
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    // En cas d'erreur (NACK, timeout, etc.), redémarrer l'écoute.
    HAL_I2C_EnableListen_IT(hi2c);
}
