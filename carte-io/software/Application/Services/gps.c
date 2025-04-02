/*
 * gps.c
 *
 *  Created on: Dec 18, 2024
 *      Author: dimercur
 */


#include "stm32u5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Constantes */
#define LPUART1_RX_BUFFER_SIZE 512 // Taille du buffer DMA circulaire
#define LINE_TERMINATOR 0x0A       // Fin de ligne ASCII (LF)

/* Handlers */
static UART_HandleTypeDef hlpuart1;
static DMA_HandleTypeDef hdma_lpuart1_rx;

/* Buffers DMA et variables */
static uint8_t dmaRxBuffer[LPUART1_RX_BUFFER_SIZE];
static volatile uint16_t dmaReadIndex = 0;

/* FreeRTOS */
static SemaphoreHandle_t xLineReadySemaphore;

/* Prototypes */
void SystemClock_Config(void);
static void LPUART1_Init(void);
static void DMA_LPUART1_Init(void);
static void vGPSReceptionTask(void *pvParameters);
static void Custom_UART_RxHalfCompleteCallback(UART_HandleTypeDef *huart);
static void Custom_UART_RxCompleteCallback(UART_HandleTypeDef *huart);
static char *RetrieveGPSLine(void);

/* Fonction principale */
int gps_init(void) {
    //HAL_Init();
    //SystemClock_Config();

    /* Initialisation LPUART1 et DMA */
    LPUART1_Init();
    DMA_LPUART1_Init();

    /* Création des sémaphores */
    xLineReadySemaphore = xSemaphoreCreateBinary();
    if (xLineReadySemaphore == NULL) {
        // Gestion erreur : impossible de créer le sémaphore
        while (1);
    }

    /* Création de la tâche FreeRTOS */
    xTaskCreate(vGPSReceptionTask, "GPSReception", 256, NULL, 2, NULL);

    /* Démarrage du scheduler FreeRTOS */
    vTaskStartScheduler();

    while (1);
}

/* Initialisation de LPUART1 */
static void LPUART1_Init(void) {
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;  // LPUART1_TX (PG7) et LPUART1_RX (PG8)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 9600;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OverSampling = UART_OVERSAMPLING_16;

    /* Activer l'utilisation des callbacks personnalisés */
    HAL_UART_RegisterCallback(&hlpuart1, HAL_UART_RX_HALFCOMPLETE_CB_ID, Custom_UART_RxHalfCompleteCallback);
    HAL_UART_RegisterCallback(&hlpuart1, HAL_UART_RX_COMPLETE_CB_ID, Custom_UART_RxCompleteCallback);

    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        // Gestion erreur
        while (1);
    }
}

///* Initialisation du DMA pour LPUART1 */
//void DMA_LPUART1_Init(void) {
//    __HAL_RCC_DMA1_CLK_ENABLE();
//
//    hdma_lpuart1_rx.Instance = DMA1_Stream2;
//    hdma_lpuart1_rx.Init.Channel = DMA_CHANNEL_5;
//    hdma_lpuart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//    hdma_lpuart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//    hdma_lpuart1_rx.Init.MemInc = DMA_MINC_ENABLE;
//    hdma_lpuart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    hdma_lpuart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    hdma_lpuart1_rx.Init.Mode = DMA_CIRCULAR;  // Mode circulaire
//    hdma_lpuart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
//    hdma_lpuart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//
//    HAL_DMA_Init(&hdma_lpuart1_rx);
//    __HAL_LINKDMA(&hlpuart1, hdmarx, hdma_lpuart1_rx);
//
//    /* Démarrage du DMA */
//    HAL_UART_Receive_DMA(&hlpuart1, dmaRxBuffer, LPUART1_RX_BUFFER_SIZE);
//}

/* Callback personnalisé : demi-completion du DMA */
static void Custom_UART_RxHalfCompleteCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        xSemaphoreGiveFromISR(xLineReadySemaphore, NULL);
    }
}

/* Callback personnalisé : completion totale du DMA */
static void Custom_UART_RxCompleteCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == LPUART1) {
        xSemaphoreGiveFromISR(xLineReadySemaphore, NULL);
    }
}

/* Tâche FreeRTOS pour réception GPS */
static void vGPSReceptionTask(void *pvParameters) {
	/* Démarrage du DMA */
	HAL_UART_Receive_DMA(&hlpuart1, dmaRxBuffer, LPUART1_RX_BUFFER_SIZE);

    while (1) {
        /* Attente d'une ligne complète */
        if (xSemaphoreTake(xLineReadySemaphore, pdMS_TO_TICKS(100)) == pdPASS) {
            char *gpsLine = RetrieveGPSLine();
            if (gpsLine != NULL) {
                /* Traiter la ligne GPS reçue */
                printf("GPS Line: %s\n", gpsLine);
                free(gpsLine);
            }
        }
    }
}

/* Fonction pour extraire une ligne GPS complète */
static char *RetrieveGPSLine(void) {
    uint16_t currentIndex = dmaReadIndex;
    uint16_t lineStartIndex = currentIndex;
    uint16_t lineLength = 0;
    char *line = NULL;

    while (currentIndex != (dmaReadIndex + LPUART1_RX_BUFFER_SIZE - 1) % LPUART1_RX_BUFFER_SIZE) {
        lineLength++;
        line = realloc(line, lineLength);
        if (line == NULL) {
            // Gestion erreur : échec de realloc
            return NULL;
        }

        line[lineLength - 1] = dmaRxBuffer[currentIndex];

        if (dmaRxBuffer[currentIndex] == LINE_TERMINATOR) {
            /* Ajouter un '\0' pour terminer la chaîne */
            line = realloc(line, lineLength + 1);
            if (line == NULL) {
                return NULL;
            }
            line[lineLength] = '\0';

            /* Mettre à jour dmaReadIndex */
            dmaReadIndex = (currentIndex + 1) % LPUART1_RX_BUFFER_SIZE;
            return line;
        }

        currentIndex = (currentIndex + 1) % LPUART1_RX_BUFFER_SIZE;
    }

    free(line); // Libérer la mémoire en cas de ligne incomplète
    return NULL;
}
