/*
 * uart_drv.cpp
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur
 */

//#include <Drv/uart_drv.h>
//
//UartDriver::UartDriver() {
//	// TODO Auto-generated constructor stub
//
//}
//
//UartDriver::~UartDriver() {
//	// TODO Auto-generated destructor stub
//}

//#pragma once
#include "stm32u5xx_hal.h"
#include <array>

template <typename Derived>
class UARTBase {
protected:
    UART_HandleTypeDef* huart_;
    bool usingDma_;

public:
    UARTBase(UART_HandleTypeDef* huart, bool useDma)
        : huart_(huart), usingDma_(useDma) {
        RegisterInstance();
    }

    virtual ~UARTBase() {
        UnregisterInstance();
    }

    bool Transmit(const uint8_t* data, size_t size) {
        HAL_StatusTypeDef status;
        if (usingDma_) {
            status = HAL_UART_Transmit_DMA(huart_, data, static_cast<uint16_t>(size));
        } else {
            status = HAL_UART_Transmit_IT(huart_, data, static_cast<uint16_t>(size));
        }
        return (status == HAL_OK);
    }

    bool Receive(uint8_t* buffer, size_t size) {
        HAL_StatusTypeDef status;
        if (usingDma_) {
            status = HAL_UART_Receive_DMA(huart_, buffer, static_cast<uint16_t>(size));
        } else {
            status = HAL_UART_Receive_IT(huart_, buffer, static_cast<uint16_t>(size));
        }
        return (status == HAL_OK);
    }

    // Call these from HAL callbacks in your main code
    static void HandleTxComplete(UART_HandleTypeDef* huart) {
        ForMatchingInstance(huart, [](Derived& instance) {
            instance.OnTxComplete();
        });
    }

    static void HandleRxComplete(UART_HandleTypeDef* huart) {
        ForMatchingInstance(huart, [](Derived& instance) {
            instance.OnRxComplete();
        });
    }

    static void HandleError(UART_HandleTypeDef* huart) {
    	auto code = huart->ErrorCode;
        ForMatchingInstance(huart, [code](Derived& instance) {
            instance.OnError(code);
        });
    }

protected:
    // These can be overridden in derived classes
    void OnTxComplete() {}
    void OnRxComplete() {}
    void OnError(uint32_t errorCode) { (void)errorCode; }

private:
    static constexpr int MAX_INSTANCES = 4;  // Adjust based on your needs
    static std::array<Derived*, MAX_INSTANCES> instances_;
    static int instanceCount_;

    void RegisterInstance() {
        if (instanceCount_ < MAX_INSTANCES) {
            instances_[instanceCount_++] = static_cast<Derived*>(this);
        }
    }

    void UnregisterInstance() {
        for (int i = 0; i < instanceCount_; i++) {
            if (instances_[i] == static_cast<Derived*>(this)) {
                instances_[i] = instances_[instanceCount_ - 1];
                instanceCount_--;
                break;
            }
        }
    }

    template <typename Action>
    static void ForMatchingInstance(UART_HandleTypeDef* huart, Action action) {
        for (int i = 0; i < instanceCount_; i++) {
            if (instances_[i]->huart_ == huart) {
                action(*instances_[i]);
                break;
            }
        }
    }
};

// Static members initialization
template <typename Derived>
std::array<Derived*, UARTBase<Derived>::MAX_INSTANCES> UARTBase<Derived>::instances_ = {};

template <typename Derived>
int UARTBase<Derived>::instanceCount_ = 0;
