/*
 * uart_drv.cpp
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#include <Drivers/uartdriver.h>

#include "debug.h"
#include <cstring>

// Constructeur
UartDriver::UartDriver(UART_HandleTypeDef* huart): uartHandler_(huart) {

	/*
	 * /!\ Attention : Le callback HAL_UART_MSPINIT_CB_ID et HAL_UART_MSPDEINIT_CB_ID sont
	 * en partie détournés de leur fonction dans HAL
	 * Ils servent ici a stocker la reference vers l'instance (objet) en cours pour
	 * pouvoir le retrouver lorsque les fonctions statiques de la classe sont appelées
	 * (Callback liés au timer, aux fin de transmission et de reception notamment)
	 *
	 * HAL_UART_MSPINIT_CB_ID contient la reference à l'instance dans tout les cas d'usage
	 * SAUF lors de l'appel à la fonction HAL_UART_Init où il contient la reference vers la fonction
	 * d'initialisation HW
	 *
	 * HAL_UART_MSPDEINIT_CB_ID contient la reference à l'instance uniquement dans le cas de l'appel
	 * à la fonction HAL_UART_Init. Dans tous les autres cas, elle contient la reference vers
	 * la fonction de de-initialisation HW
	 */
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPINIT_CB_ID, (pUART_CallbackTypeDef)this);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPDEINIT_CB_ID, hwDeInit);

	HAL_UART_DeInit(uartHandler_);

	// Creation des semaphores
	txCompleteSemaphore_ = xSemaphoreCreateBinary();
	assert_param(txCompleteSemaphore_ != nullptr);

	rxCompleteSemaphore_ = xSemaphoreCreateBinary();
	assert_param(rxCompleteSemaphore_ != nullptr);

	/*
	 * le semaphore TX doit être à 1 pour pouvoir être pris
	 * par la méthode write en entrant: cela évite la reantrance dans la fonction write
	 * tant qu'elle n'a pas finie
	 *
	 * A l'inverse, le semaphore RX doit être à l'etat 0 en entrant dans la méthode
	 * read pour bloquer tant que les données n'ont pas été reçues (et donc que le
	 * semaphore RX soit produit)
	 */
	xSemaphoreGive(txCompleteSemaphore_);
	vQueueAddToRegistry(txCompleteSemaphore_, "TX Complete");
	vQueueAddToRegistry(rxCompleteSemaphore_, "RX Complete");
}

UartDriver::~UartDriver() {
	vSemaphoreDelete(txCompleteSemaphore_);
	vSemaphoreDelete(rxCompleteSemaphore_);

	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPDEINIT_CB_ID, hwDeInit);
	HAL_UART_DeInit(uartHandler_);
}

// Configuration
HAL_StatusTypeDef UartDriver::configure(uint32_t baudrate,
		UartDriver_ModeTypeDef tx_mode, UartDriver_ModeTypeDef rx_mode,
		uint8_t *circular_buffer = nullptr, uint32_t circular_buffer_size = 0, uint32_t timer_delay = 100) {
	assert_param(
			(tx_mode == MODE_POLLING) || (tx_mode == MODE_IRQ) || (tx_mode == MODE_DMA));
	assert_param(
			(tx_mode == MODE_POLLING) || (tx_mode == MODE_IRQ) || (tx_mode == MODE_DMA) || (tx_mode == MODE_CIRCULAR_DMA));

	if (rx_mode == MODE_CIRCULAR_DMA) {
		assert_param(circular_buffer != nullptr);
		assert_param(circular_buffer_size != 0);

		circularBuffer_ = circular_buffer;
		circularBufferSize_ = circular_buffer_size;
		dmaReadIndex_ = 0;
		timerDelay_ = timer_delay;
		readInProgress_ = false;
	}

	txMode_ = tx_mode;
	rxMode_ = rx_mode;

	/*
	 * /!\ Attention : Le callback HAL_UART_MSPINIT_CB_ID et HAL_UART_MSPDEINIT_CB_ID sont
	 * en partie détournés de leur fonction dans HAL
	 * Ils servent ici a stocker la reference vers l'instance (objet) en cours pour
	 * pouvoir le retrouver lorsque les fonctions statiques de la classe sont appelées
	 * (Callback liés au timer, aux fin de transmission et de reception notamment)
	 *
	 * HAL_UART_MSPINIT_CB_ID contient la reference à l'instance dans tout les cas d'usage
	 * SAUF lors de l'appel à la fonction HAL_UART_Init où il contient la reference vers la fonction
	 * d'initialisation HW
	 *
	 * HAL_UART_MSPDEINIT_CB_ID contient la reference à l'instance uniquement dans le cas de l'appel
	 * à la fonction HAL_UART_Init. Dans tous les autres cas, elle contient la reference vers
	 * la fonction de de-initialisation HW
	 */
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPINIT_CB_ID, hwInit);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPDEINIT_CB_ID, (pUART_CallbackTypeDef)this);

	uartHandler_->Instance = USART1;
	uartHandler_->Init.BaudRate = baudrate;
	uartHandler_->Init.WordLength = UART_WORDLENGTH_8B;
	uartHandler_->Init.StopBits = UART_STOPBITS_1;
	uartHandler_->Init.Parity = UART_PARITY_NONE;
	uartHandler_->Init.Mode = UART_MODE_TX_RX;
	uartHandler_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uartHandler_->Init.OverSampling = UART_OVERSAMPLING_16;
	uartHandler_->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uartHandler_->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	uartHandler_->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(uartHandler_) != HAL_OK)
		return HAL_ERROR;

	if (HAL_UARTEx_SetTxFifoThreshold(uartHandler_,
			UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		return HAL_ERROR;

	if (HAL_UARTEx_SetRxFifoThreshold(uartHandler_,
			UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		return HAL_ERROR;

	if (HAL_UARTEx_DisableFifoMode(uartHandler_) != HAL_OK)
		return HAL_ERROR;

	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_TX_COMPLETE_CB_ID, txCompleteCallback);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_RX_HALFCOMPLETE_CB_ID, rxHalfCompleteCallback);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_RX_COMPLETE_CB_ID, rxCompleteCallback);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_ERROR_CB_ID, errorCallback);

	/*
	 * /!\ Attention : Le callback HAL_UART_MSPINIT_CB_ID et HAL_UART_MSPDEINIT_CB_ID sont
	 * en partie détournés de leur fonction dans HAL
	 * Ils servent ici a stocker la reference vers l'instance (objet) en cours pour
	 * pouvoir le retrouver lorsque les fonctions statiques de la classe sont appelées
	 * (Callback liés au timer, aux fin de transmission et de reception notamment)
	 *
	 * HAL_UART_MSPINIT_CB_ID contient la reference à l'instance dans tout les cas d'usage
	 * SAUF lors de l'appel à la fonction HAL_UART_Init où il contient la reference vers la fonction
	 * d'initialisation HW
	 *
	 * HAL_UART_MSPDEINIT_CB_ID contient la reference à l'instance uniquement dans le cas de l'appel
	 * à la fonction HAL_UART_Init. Dans tous les autres cas, elle contient la reference vers
	 * la fonction de de-initialisation HW
	 */
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPINIT_CB_ID, (pUART_CallbackTypeDef)this);
	HAL_UART_RegisterCallback(uartHandler_,
			HAL_UART_MSPDEINIT_CB_ID, hwDeInit);

	if (rxMode_ == MODE_CIRCULAR_DMA) {
		if (timerDelay_ != 0) {
			periodicTimer_ = xTimerCreate("UART_Timer",           // Nom du timer
					pdMS_TO_TICKS(timer_delay),    // Période en ticks ( ex 500 ms)
					pdTRUE,       // Auto-reload (pdTRUE = répète, pdFALSE = unique)
					(void*) this,             // ID du timer (facultatif)
					UartTimerCallback      // Fonction callback
			);

			//vQueueAddToRegistry(periodicTimer_,"Timer");
		}

		// Demarrage de la DMA circulaire
		if (HAL_UARTEx_ReceiveToIdle_DMA(uartHandler_, circularBuffer_, circular_buffer_size) != HAL_OK)
			PANIC("Impossible de lancer la DMA circulaire");
	}

	return HAL_OK;
}

void UartDriver::onHWInitEvent(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	DMA_NodeConfTypeDef NodeConfig;

	if(uartHandler_->Instance==USART1)
	{
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */

		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
		PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			PANIC("Clock configuration for UART1 failed");

		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}

	if (txMode_ == MODE_DMA) {
		if (uartHandler_->Instance == USART1) {
			/* Peripheral clock enable */
			__HAL_RCC_GPDMA1_CLK_ENABLE();

			/* GPDMA1 interrupt Init */
			HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

			/* USER CODE BEGIN GPDMA1_Init 1 */

			/* USART1 DMA Init */
			/* GPDMA1_REQUEST_USART1_TX Init */
			handle_GPDMA1_Channel_TX_.Instance = GPDMA1_Channel1;
			handle_GPDMA1_Channel_TX_.Init.Request = GPDMA1_REQUEST_USART1_TX;
			handle_GPDMA1_Channel_TX_.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
			handle_GPDMA1_Channel_TX_.Init.Direction = DMA_MEMORY_TO_PERIPH;
			handle_GPDMA1_Channel_TX_.Init.SrcInc = DMA_SINC_INCREMENTED;
			handle_GPDMA1_Channel_TX_.Init.DestInc = DMA_DINC_FIXED;
			handle_GPDMA1_Channel_TX_.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
			handle_GPDMA1_Channel_TX_.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
			handle_GPDMA1_Channel_TX_.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
			handle_GPDMA1_Channel_TX_.Init.SrcBurstLength = 1;
			handle_GPDMA1_Channel_TX_.Init.DestBurstLength = 1;
			handle_GPDMA1_Channel_TX_.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
			handle_GPDMA1_Channel_TX_.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
			handle_GPDMA1_Channel_TX_.Init.Mode = DMA_NORMAL;
			if (HAL_DMA_Init(&handle_GPDMA1_Channel_TX_) != HAL_OK)
			{
				PANIC("DMA init error (UART1)");
			}

			__HAL_LINKDMA(uartHandler_, hdmatx, handle_GPDMA1_Channel_TX_);

			if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel_TX_, DMA_CHANNEL_NPRIV) != HAL_OK)
			{
				PANIC("DMA configuration error (UART1)");
			}
		}
	}

	if ((rxMode_ == MODE_DMA) || (rxMode_ == MODE_CIRCULAR_DMA)) {
		if (uartHandler_->Instance == USART1) {
			/* Peripheral clock enable */
			__HAL_RCC_GPDMA1_CLK_ENABLE();

			/* GPDMA1 interrupt Init */
			HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

			/* USER CODE BEGIN GPDMA1_Init 1 */

			memset(&NodeConfig, 0, sizeof(NodeConfig));
			memset(&List_GPDMA1_Channel_RX_, 0, sizeof(List_GPDMA1_Channel_RX_));

			/* GPDMA1_REQUEST_USART1_RX Init */
			NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
			NodeConfig.Init.Request = GPDMA1_REQUEST_USART1_RX;
			NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
			NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
			NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
			NodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
			NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
			NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
			NodeConfig.Init.SrcBurstLength = 1;
			NodeConfig.Init.DestBurstLength = 1;
			NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
			NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
			NodeConfig.Init.Mode = DMA_NORMAL;
			NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
			NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
			NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
			if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel_RX_) != HAL_OK)
			{
				PANIC("DMA node init error (UART1)");
			}

			if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel_RX_, NULL, &Node_GPDMA1_Channel_RX_) != HAL_OK)
			{
				PANIC("DMA node insert error (UART1)");
			}

			if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel_RX_) != HAL_OK)
			{
				PANIC("DMA list setcircularmode error (UART1)");
			}

			handle_GPDMA1_Channel_RX_.Instance = GPDMA1_Channel0;
			handle_GPDMA1_Channel_RX_.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
			handle_GPDMA1_Channel_RX_.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
			handle_GPDMA1_Channel_RX_.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT0;
			handle_GPDMA1_Channel_RX_.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
			handle_GPDMA1_Channel_RX_.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
			if (HAL_DMAEx_List_Init(&handle_GPDMA1_Channel_RX_) != HAL_OK)
			{
				PANIC("DMA list init error (UART1)");
			}

			if (HAL_DMAEx_List_LinkQ(&handle_GPDMA1_Channel_RX_, &List_GPDMA1_Channel_RX_) != HAL_OK)
			{
				PANIC("DMA list link error (UART1)");
			}

			__HAL_LINKDMA(uartHandler_, hdmarx, handle_GPDMA1_Channel_RX_);

			if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel_RX_, DMA_CHANNEL_NPRIV) != HAL_OK)
			{
				PANIC("DMA configuration error (UART1)");
			}
		}
	}

	if ((txMode_ != MODE_POLLING) && (rxMode_ != MODE_POLLING)) {
		if (uartHandler_->Instance == USART1) {
			/* UART1 interrupt Init */
			HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
			HAL_NVIC_EnableIRQ(USART1_IRQn);
		}
	}
}

void UartDriver::onHWDeInitEvent(void) {
	if (uartHandler_->Instance==USART1) {
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

		/* USART1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	}
}

HAL_StatusTypeDef UartDriver::write(
		const uint8_t *data,
		uint16_t size,
		uint32_t timeout,
		bool deleteBuffer) {
	HAL_StatusTypeDef status = HAL_ERROR;
	TickType_t timeoutFreertos = portMAX_DELAY;

	if (txMode_ != MODE_POLLING) {
		if (timeout!=portMAX_DELAY)
			timeoutFreertos = pdMS_TO_TICKS(timeout);

		if (xSemaphoreTake(txCompleteSemaphore_, timeoutFreertos) == pdTRUE) {
			// Enregistrement de la demande de liberation mémoire en fin de transfert
			deleteBufferAfterTX_ = deleteBuffer;
			txBuffer_ = data;

			if (txMode_ == MODE_IRQ)
				status = HAL_UART_Transmit_IT(uartHandler_,
						data, size);
			else
				status = HAL_UART_Transmit_DMA(uartHandler_,
						data, size);
		} else
			status = HAL_TIMEOUT;
	} else
		status = HAL_UART_Transmit(uartHandler_, data, size,timeout);

	return status;
}

HAL_StatusTypeDef UartDriver::read(uint8_t *data, uint16_t size,
		uint32_t timeout) {
	HAL_StatusTypeDef status = HAL_ERROR;
	BaseType_t semStatus = pdFALSE;

	if ((rxMode_ != MODE_POLLING) && (rxMode_ != MODE_CIRCULAR_DMA)) {
		if (rxMode_ == MODE_IRQ)
			status = HAL_UART_Receive_IT(uartHandler_,
					data, size);
		else
			status = HAL_UART_Receive_DMA(uartHandler_,
					data, size);

		if (status == HAL_OK) {
			if (timeout != portMAX_DELAY) {
				if (xSemaphoreTake(rxCompleteSemaphore_,pdMS_TO_TICKS(timeout)) != pdTRUE)
					status = HAL_TIMEOUT;
			} else {
				// Attente infinie tant que le semaphore n'est pas produit
				while (semStatus != pdTRUE) {
					semStatus = xSemaphoreTake(rxCompleteSemaphore_,	portMAX_DELAY);
				}
			}
		}
	} else if (rxMode_ == MODE_CIRCULAR_DMA) {
		outputBuffer_ = data;
		outputSize_ = size;
		writeIndex_ = 0;
		readInProgress_ = true;

		// Démarrage du timer / lecture periodique
		BaseType_t timerStat = xTimerStart(periodicTimer_,0);
		assert_param(timerStat == pdPASS);

		// on part du principe que tout va bien se passer
		status = HAL_OK;

		// attente du semaphore de fin de lecture
		if (timeout != portMAX_DELAY) {
			if (xSemaphoreTake(rxCompleteSemaphore_, pdMS_TO_TICKS(timeout))!=pdTRUE)
				status = HAL_TIMEOUT;
		} else {
			// Attente infinie tant que le semaphore n'est pas produit
			while (semStatus != pdTRUE) {
				semStatus = xSemaphoreTake(rxCompleteSemaphore_,	portMAX_DELAY);
			}
		}
	} else { // rxMode == MODE_POLLING
		status = HAL_UART_Receive(uartHandler_, data, size,
				timeout);
	}

	return status;
}

bool UartDriver::proceedCircularDMA(uint32_t currentDMAIndex) {
	if (readInProgress_) {
		if (writeIndex_ >= outputSize_)
			return true; // Si déjà rempli, ne rien faire

		size_t dmaWriteIndex = (size_t) circularBufferSize_ - (size_t) currentDMAIndex;
		// L'index DMA va de la taille du buffer à 0 (décrément)
		// ainsi, si le buffer a une taille de 50 et l'index vaut 48
		// il n'y a que 50-48 =2 octets dans le buffer

		size_t availableData =
				(dmaWriteIndex >= dmaReadIndex_) ?
						(dmaWriteIndex - dmaReadIndex_) :
						(circularBufferSize_ - dmaReadIndex_ + dmaWriteIndex);

		while (availableData > 0 && writeIndex_ < outputSize_) {
			outputBuffer_[writeIndex_] = circularBuffer_[dmaReadIndex_];
			dmaReadIndex_ = (dmaReadIndex_ + 1) % circularBufferSize_;
			writeIndex_++;
			availableData--;

			if (writeIndex_ == outputSize_) {
				// On a reçu nos données, arret du timer periodique et on indique que l'on n'est plus en phase de reception
				xTimerStop(periodicTimer_, 0);
				readInProgress_ = false;
				return true;
			}
		}
	}

	return false;
}

/*
 * Ces fonctions sont appelée en fait sous interruption, sauf
 * onRXEvent(UART_RX_TIMER)
 */
void UartDriver::onTXEvent(UART_EventTypedef event) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// liberation mémoire du buffer TX si demandé lors de l'envoi
	if ((deleteBufferAfterTX_) && (txBuffer_))
		delete(txBuffer_);

	/* Liberation du semaphore TX */
	xSemaphoreGiveFromISR(txCompleteSemaphore_, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The
	 actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UartDriver::onRXEvent(UART_EventTypedef event) {
	bool status;

	if ((rxMode_ == MODE_CIRCULAR_DMA) && (readInProgress_ == true)) {
		status = proceedCircularDMA(__HAL_DMA_GET_COUNTER(uartHandler_->hdmarx));
		if (status)
			xSemaphoreGive(rxCompleteSemaphore_);
	} else {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		/* Liberation du semaphore RX */
		xSemaphoreGiveFromISR(rxCompleteSemaphore_, &xHigherPriorityTaskWoken);

		/* Yield if xHigherPriorityTaskWoken is true. The  actual macro used here is port specific. */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void UartDriver::onErrorEvent(UART_EventTypedef event) {
	assert_param(false);
}

// Callbacks statiques pour HAL
void UartDriver::txCompleteCallback(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspInitCallback);
	if (instance) {
		instance->onTXEvent(UART_TX_COMPLETE);
	}
}

void UartDriver::rxCompleteCallback(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspInitCallback);
	if (instance) {
		instance->onRXEvent(UART_RX_COMPLETE);
	}
}

void UartDriver::rxHalfCompleteCallback(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspInitCallback);
	if (instance) {
		instance->onRXEvent(UART_RX_HALFCOMPLETE);
	}
}

void UartDriver::errorCallback(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspInitCallback);
	if (instance) {
		instance->onErrorEvent(UART_ERROR);
	}
}

void UartDriver::UartTimerCallback(TimerHandle_t xTimer) {
	UartDriver *instance = static_cast<UartDriver*>(pvTimerGetTimerID(xTimer));
	if (instance) {
		instance->onRXEvent(UART_RX_TIMER);
	}
}

void UartDriver::hwInit(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspDeInitCallback);
	if (instance) {
		instance->onHWInitEvent();
	}
}

void UartDriver::hwDeInit(UART_HandleTypeDef *huart) {
	// Conversion unsafe, mais sous contrôle
	UartDriver *instance = reinterpret_cast<UartDriver*>(huart->MspInitCallback);
	if (instance) {
		instance->onHWDeInitEvent();
	}
}
