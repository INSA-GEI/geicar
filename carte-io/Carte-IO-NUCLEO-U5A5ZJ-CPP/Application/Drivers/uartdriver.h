/*
 * uart_drv.h
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#ifndef BASE_UART_H
#define BASE_UART_H

#include "stm32u5xx.h"
#include <functional>

// Structure étendue pour ajouter un pointeur utilisateur
typedef struct {
    UART_HandleTypeDef huart;
    void* pUserData;
} ExtendedUartHandle;

class UartDriver {
public:
    // Constructeur prenant un handler standard HAL
    UartDriver(UART_HandleTypeDef& huart);

    // Configuration de l'UART
    void configure(uint32_t baudrate);

    // Transmission et réception
    HAL_StatusTypeDef sendDataIT(uint8_t* data, uint16_t size);
    HAL_StatusTypeDef receiveDataIT(uint8_t* buffer, uint16_t size);

    HAL_StatusTypeDef sendDataPolling(uint8_t* data, uint16_t size, uint32_t timeout);
    HAL_StatusTypeDef receiveDataPolling(uint8_t* buffer, uint16_t size, uint32_t timeout);

    // Enregistrement des callbacks
    void setTxCompleteCallback(std::function<void()> callback);
    void setRxCompleteCallback(std::function<void()> callback);
    void setErrorCallback(std::function<void()> callback);

private:
    ExtendedUartHandle _huart;  // Attribut privé

    // Callbacks stockés avec std::function
    std::function<void()> _txCompleteCallback;
    std::function<void()> _rxCompleteCallback;
    std::function<void()> _errorCallback;

    // Callbacks statiques pour HAL
    static void txCompleteCallback(UART_HandleTypeDef* huart);
    static void rxCompleteCallback(UART_HandleTypeDef* huart);
    static void errorCallback(UART_HandleTypeDef* huart);
};

#endif // BASE_UART_H
