/*
 * rplidar-c1.c
 *
 *  Created on: Dec 18, 2024
 *      Author: dimercur
 */

#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* Constantes */
#define UART4_RX_BUFFER_SIZE 512  // Taille du buffer DMA circulaire
#define RPLIDAR_FRAME_SIZE 5      // Taille d'une trame standard du RPLIDAR C1
#define RPLIDAR_START_FLAG 0xA5   // Flag de démarrage de commande
#define RPLIDAR_RESPONSE_FLAG 0x5A

/* Handlers */
static UART_HandleTypeDef huart4;
static DMA_HandleTypeDef hdma_uart4_rx;

/* Buffers DMA et variables */
static uint8_t dmaRxBuffer[UART4_RX_BUFFER_SIZE];
static volatile uint16_t dmaReadIndex = 0;

/* FreeRTOS */
static SemaphoreHandle_t xHalfCompleteSemaphore;
static SemaphoreHandle_t xCompleteSemaphore;

/* Prototypes */
void SystemClock_Config(void);
static void UART4_Init(void);
static void DMA_UART4_Init(void);
static void vLidarTask(void *pvParameters);
static void SendRPLidarCommand(uint8_t cmd);
static void ProcessLidarFrame(uint8_t *frame);
static void Custom_UART_RxHalfCompleteCallback(UART_HandleTypeDef *huart);
static void Custom_UART_RxCompleteCallback(UART_HandleTypeDef *huart);
static void ProcessCircularBuffer(uint16_t startIdx, uint16_t endIdx);

/* Fonction principale */
int rplidar_c1_init(void) {
    //HAL_Init();
    //SystemClock_Config();

    /* Initialisation UART et DMA */
    UART4_Init();
    DMA_UART4_Init();

    /* Création des sémaphores */
    xHalfCompleteSemaphore = xSemaphoreCreateBinary();
    xCompleteSemaphore = xSemaphoreCreateBinary();
    if (xHalfCompleteSemaphore == NULL || xCompleteSemaphore == NULL) {
        // Gestion erreur : impossible de créer les sémaphores
        while (1);
    }

    /* Création de la tâche FreeRTOS */
    xTaskCreate(vLidarTask, "LidarTask", 256, NULL, 2, NULL);

    /* Démarrage du scheduler FreeRTOS */
    vTaskStartScheduler();

    while (1);
}

/* Initialisation de l'UART4 */
static void UART4_Init(void) {
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // UART4_TX (PA0) et UART4_RX (PA1)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;

    /* Activer l'utilisation des callbacks personnalisés */
    HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_HALFCOMPLETE_CB_ID, Custom_UART_RxHalfCompleteCallback);
    HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, Custom_UART_RxCompleteCallback);

    if (HAL_UART_Init(&huart4) != HAL_OK) {
        // Gestion erreur
        while (1);
    }
}

///* Initialisation du DMA pour UART4 */
//void DMA_UART4_Init(void) {
//    __HAL_RCC_DMA1_CLK_ENABLE();
//
//    hdma_uart4_rx.Instance = DMA1_Stream0;
//    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
//    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;  // Mode circulaire
//    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
//    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//
//    HAL_DMA_Init(&hdma_uart4_rx);
//    __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);
//
//    /* Démarrage du DMA */
//    HAL_UART_Receive_DMA(&huart4, dmaRxBuffer, UART4_RX_BUFFER_SIZE);
//}

/* Envoi d'une commande au RPLIDAR */
static void SendRPLidarCommand(uint8_t cmd) {
    uint8_t command[2] = {RPLIDAR_START_FLAG, cmd};
    HAL_UART_Transmit(&huart4, command, sizeof(command), HAL_MAX_DELAY);
}

/* Callback personnalisé : demi-completion du DMA */
static void Custom_UART_RxHalfCompleteCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        xSemaphoreGiveFromISR(xHalfCompleteSemaphore, NULL);
    }
}

/* Callback personnalisé : completion totale du DMA */
static void Custom_UART_RxCompleteCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        xSemaphoreGiveFromISR(xCompleteSemaphore, NULL);
    }
}

/* Tâche FreeRTOS pour traiter les trames du RPLIDAR */
static void vLidarTask(void *pvParameters) {
	/* Démarrage du DMA */
	HAL_UART_Receive_DMA(&huart4, dmaRxBuffer, UART4_RX_BUFFER_SIZE);

    /* Envoi de la commande pour démarrer le balayage */
    SendRPLidarCommand(0x20);  // Commande pour démarrer le scan

    while (1) {
        /* Attente des sémaphores */
        if (xSemaphoreTake(xHalfCompleteSemaphore, pdMS_TO_TICKS(100)) == pdPASS) {
            ProcessCircularBuffer(0, UART4_RX_BUFFER_SIZE / 2);
        }

        if (xSemaphoreTake(xCompleteSemaphore, pdMS_TO_TICKS(100)) == pdPASS) {
            ProcessCircularBuffer(UART4_RX_BUFFER_SIZE / 2, UART4_RX_BUFFER_SIZE);
        }
    }
}

/* Traitement des données dans le buffer circulaire */
static void ProcessCircularBuffer(uint16_t startIdx, uint16_t endIdx) {
    uint16_t currentIdx = startIdx;

    while (currentIdx != endIdx) {
        if ((endIdx - currentIdx) >= RPLIDAR_FRAME_SIZE) {
            /* Extraction d'une trame */
            uint8_t frame[RPLIDAR_FRAME_SIZE];
            memcpy(frame, &dmaRxBuffer[currentIdx], RPLIDAR_FRAME_SIZE);

            /* Traitement de la trame */
            ProcessLidarFrame(frame);

            /* Avancer l'index */
            currentIdx += RPLIDAR_FRAME_SIZE;
        } else {
            break;  // Pas assez de données pour une trame complète
        }
    }
}

/* Traitement d'une trame valide */
static void ProcessLidarFrame(uint8_t *frame) {
    if ((frame[0] & 0x01) == 0) {
        // Donnée invalide (selon le protocole RPLIDAR)
        return;
    }

    uint16_t angle = (((frame[1] >> 1) | (frame[2] << 7)) & 0x7FFF);
    uint16_t distance = (frame[3] | (frame[4] << 8));

    printf("Angle: %.2f°, Distance: %.2f mm\n", angle / 64.0, distance / 4.0);
}


