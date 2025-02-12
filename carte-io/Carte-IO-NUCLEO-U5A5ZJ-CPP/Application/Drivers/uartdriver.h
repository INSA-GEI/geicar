/*
 * uart_drv.h
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#ifndef BASE_UART_H
#define BASE_UART_H

#include "stm32u5xx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

typedef enum {
	MODE_POLLING = 0x00U, MODE_IRQ, MODE_DMA, MODE_CIRCULAR_DMA
} UartDriver_ModeTypeDef;

class UartDriver {
public:
	// Constructeur prenant un handler standard HAL
	UartDriver(UART_HandleTypeDef* huart);
	// Destructeur
	~UartDriver();

	// Configuration de l'UART
	HAL_StatusTypeDef configure(uint32_t baudrate,
			UartDriver_ModeTypeDef tx_mode, UartDriver_ModeTypeDef rx_mode,
			uint8_t *circular_buffer, uint32_t circular_buffer_size,
			uint32_t timer_delay);

	HAL_StatusTypeDef configure(uint32_t baudrate) { // TX and RX in polling
		return configure(baudrate, MODE_POLLING, MODE_POLLING, nullptr, 0, 0); // <- timer_delay =0 car le timer ne sert QUE dans le cas du DMA circulaire
	}

	HAL_StatusTypeDef configure(uint32_t baudrate,
			UartDriver_ModeTypeDef tx_mode, UartDriver_ModeTypeDef rx_mode) {
		assert_param(rx_mode != MODE_CIRCULAR_DMA); // en RX DMA circulaire, il faut indiquer un buffer
		return configure(baudrate, tx_mode, rx_mode, nullptr, 0, 0); // <- timer_delay =0 car le timer ne sert QUE dans le cas du DMA circulaire
	}

	HAL_StatusTypeDef write(uint8_t *data, uint16_t size, uint32_t timeout);
	HAL_StatusTypeDef write(uint8_t *data, uint16_t size) {
		return write(data, size, (uint32_t) portMAX_DELAY);
	};

	HAL_StatusTypeDef write(const char *data, uint16_t size) {
		return write((uint8_t*) data, size);
	}

	HAL_StatusTypeDef write(const char *data, uint16_t size, uint32_t timeout) {
		return write((uint8_t*) data, size, timeout);
	}

	HAL_StatusTypeDef read(uint8_t *data, uint16_t size, uint32_t timeout);
	HAL_StatusTypeDef read(uint8_t *data, uint16_t size) {
		return read(data, size, (uint32_t) portMAX_DELAY);
	}

	HAL_StatusTypeDef read(const char *data, uint16_t size) {
		return read((uint8_t*) data, size);
	}

	HAL_StatusTypeDef read(const char *data, uint16_t size, uint32_t timeout) {
		return read((uint8_t*) data, size, timeout);
	}

private:
	typedef enum {
		UART_TX_COMPLETE = 0x00U,
		UART_TX_HALFCOMPLETE,
		UART_RX_COMPLETE,
		UART_RX_HALFCOMPLETE,
		UART_RX_TIMER,
		UART_ERROR
	} UART_EventTypedef;

	UART_HandleTypeDef *uartHandler_;

	DMA_HandleTypeDef handle_GPDMA1_Channel_TX_;
	DMA_HandleTypeDef handle_GPDMA1_Channel_RX_;

	UartDriver_ModeTypeDef txMode_ = MODE_POLLING;
	UartDriver_ModeTypeDef rxMode_ = MODE_POLLING;

	uint32_t timerDelay_ = 100;

	SemaphoreHandle_t txCompleteSemaphore_ = nullptr;
	SemaphoreHandle_t rxCompleteSemaphore_ = nullptr;

	TimerHandle_t periodicTimer_;

	/* gestion de la DMA circulaire */
	uint8_t *circularBuffer_ = nullptr;
	uint32_t circularBufferSize_ = 0;
	uint8_t *outputBuffer_ = nullptr;
	uint32_t outputSize_ = 0;
	uint32_t dmaReadIndex_ = 0;
	uint32_t writeIndex_ = 0;
	bool readInProgress_ = false;

	bool proceedCircularDMA(uint32_t currentDMAIndex);

	void onTXEvent(UART_EventTypedef event);
	void onRXEvent(UART_EventTypedef event);
	void onErrorEvent(UART_EventTypedef event);
	void onHWInitEvent(void);
	void onHWDeInitEvent(void);

	// Callbacks statiques pour HAL
	static void txCompleteCallback(UART_HandleTypeDef *huart);
	static void rxCompleteCallback(UART_HandleTypeDef *huart);
	static void rxHalfCompleteCallback(UART_HandleTypeDef *huart);
	static void errorCallback(UART_HandleTypeDef *huart);
	static void UartTimerCallback(TimerHandle_t xTimer);

	static void hwInit(UART_HandleTypeDef *huart);
	static void hwDeInit(UART_HandleTypeDef *huart);
};

#endif // BASE_UART_H
