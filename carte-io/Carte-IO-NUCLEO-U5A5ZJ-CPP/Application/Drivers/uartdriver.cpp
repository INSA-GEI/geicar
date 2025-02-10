/*
 * uart_drv.cpp
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#include <Drivers/uartdriver.h>

// Constructeur
UartDriver::UartDriver(UART_HandleTypeDef& huart) {
    // Copie des données du handler HAL
    _huart.huart = huart;
    _huart.pUserData = this;
}

// Configuration
void UartDriver::configure(uint32_t baudrate) {
    _huart.huart.Init.BaudRate = baudrate;
    _huart.huart.Init.WordLength = UART_WORDLENGTH_8B;
    _huart.huart.Init.StopBits = UART_STOPBITS_1;
    _huart.huart.Init.Parity = UART_PARITY_NONE;
    _huart.huart.Init.Mode = UART_MODE_TX_RX;
    _huart.huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    _huart.huart.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init((UART_HandleTypeDef*)&_huart);

    HAL_UART_RegisterCallback((UART_HandleTypeDef*)&_huart, HAL_UART_TX_COMPLETE_CB_ID, txCompleteCallback);
    HAL_UART_RegisterCallback((UART_HandleTypeDef*)&_huart, HAL_UART_RX_COMPLETE_CB_ID, rxCompleteCallback);
    HAL_UART_RegisterCallback((UART_HandleTypeDef*)&_huart, HAL_UART_ERROR_CB_ID, errorCallback);
}

// Transmission / Réception
HAL_StatusTypeDef UartDriver::sendDataIT(uint8_t* data, uint16_t size) {
    return HAL_UART_Transmit_IT((UART_HandleTypeDef*)&_huart, data, size);
}

HAL_StatusTypeDef UartDriver::receiveDataIT(uint8_t* buffer, uint16_t size) {
    return HAL_UART_Receive_IT((UART_HandleTypeDef*)&_huart, buffer, size);
}

HAL_StatusTypeDef UartDriver::sendDataPolling(uint8_t* data, uint16_t size, uint32_t timeout) {
    return HAL_UART_Transmit((UART_HandleTypeDef*)&_huart, data, size, timeout);
}

HAL_StatusTypeDef UartDriver::receiveDataPolling(uint8_t* buffer, uint16_t size, uint32_t timeout) {
    return HAL_UART_Receive((UART_HandleTypeDef*)&_huart, buffer, size, timeout);
}

// Enregistrement des callbacks
void UartDriver::setTxCompleteCallback(std::function<void()> callback) {
    _txCompleteCallback = callback;
}

void UartDriver::setRxCompleteCallback(std::function<void()> callback) {
    _rxCompleteCallback = callback;
}

void UartDriver::setErrorCallback(std::function<void()> callback) {
    _errorCallback = callback;
}

// Callbacks statiques pour HAL
void UartDriver::txCompleteCallback(UART_HandleTypeDef* huart) {
    ExtendedUartHandle* extHuart = reinterpret_cast<ExtendedUartHandle*>(huart);
    UartDriver* instance = static_cast<UartDriver*>(extHuart->pUserData);
    if (instance && instance->_txCompleteCallback) {
        instance->_txCompleteCallback();
    }
}

void UartDriver::rxCompleteCallback(UART_HandleTypeDef* huart) {
    ExtendedUartHandle* extHuart = reinterpret_cast<ExtendedUartHandle*>(huart);
    UartDriver* instance = static_cast<UartDriver*>(extHuart->pUserData);
    if (instance && instance->_rxCompleteCallback) {
        instance->_rxCompleteCallback();
    }
}

void UartDriver::errorCallback(UART_HandleTypeDef* huart) {
    ExtendedUartHandle* extHuart = reinterpret_cast<ExtendedUartHandle*>(huart);
    UartDriver* instance = static_cast<UartDriver*>(extHuart->pUserData);
    if (instance && instance->_errorCallback) {
        instance->_errorCallback();
    }
}
