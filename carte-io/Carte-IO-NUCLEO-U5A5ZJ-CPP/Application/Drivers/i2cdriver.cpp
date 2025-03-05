/*
 * i2cdriver.cpp
 *
 *  Created on: Jan 28, 2025
 *      Author: dimercur et chatgpt
 */

#include <Drivers/i2cdriver.h>

#include "debug.h"

I2cDriver::~I2cDriver() {
	vSemaphoreDelete(txCompleteSemaphore_);
	vSemaphoreDelete(rxCompleteSemaphore_);

	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPDEINIT_CB_ID, hwDeInit);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPDEINIT_CB_ID, hwDeInit);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPDEINIT_CB_ID, hwDeInit);

	HAL_I2C_DeInit(handler_);
}

// Configuration des bus I2C
bool I2cDriver::configure(I2C_HandleTypeDef* hi2c_internal,
		I2C_HandleTypeDef* hi2c_external,
		I2C_HandleTypeDef* hi2c_arbitrary,
		I2cDriver_ModeTypeDef mode,
		uint32_t timeout) {
	assert_param(
			(mode == MODE_POLLING) || (mode == MODE_IRQ));

	bool status = true;

	handler_ = hi2c_internal;
	mode_ = mode;
	timeout_ = timeout;

	/*****************************************************************************
	 *                                                                           *
	 *                      I2C bus Configuration                                *
	 *                                                                           *
	 *****************************************************************************/

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
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPINIT_CB_ID, (pI2C_CallbackTypeDef)this);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPDEINIT_CB_ID, hwDeInit);

	HAL_I2C_DeInit(handler_);

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
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPINIT_CB_ID, hwInit);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MSPDEINIT_CB_ID, (pI2C_CallbackTypeDef)this);

	handler_->Init.Timing = 0x00F07BFF;
	handler_->Init.OwnAddress1 = 0;
	handler_->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	handler_->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	handler_->Init.OwnAddress2 = 0;
	handler_->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	handler_->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	handler_->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(handler_) != HAL_OK)
		return false;

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(handler_, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		return false;

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(handler_, 0) != HAL_OK)
		return false;

	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MASTER_TX_COMPLETE_CB_ID, txCompleteCallback);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_MASTER_RX_COMPLETE_CB_ID, rxCompleteCallback);
	HAL_I2C_RegisterCallback(handler_,
			HAL_I2C_ERROR_CB_ID, errorCallback);

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

	if (handler_->Instance == I2C1) {
		vQueueAddToRegistry(txCompleteSemaphore_, "TX I2C Int");
		vQueueAddToRegistry(rxCompleteSemaphore_, "RX I2C Int");
	} else if (handler_->Instance == I2C2) {
		vQueueAddToRegistry(txCompleteSemaphore_, "TX I2C Ext");
		vQueueAddToRegistry(rxCompleteSemaphore_, "RX I2C Ext");
	} else { // I2C3
		vQueueAddToRegistry(txCompleteSemaphore_, "TX I2C Arb");
		vQueueAddToRegistry(rxCompleteSemaphore_, "RX I2C Arb");
	}

	return status;
}

void I2cDriver::onHWInitEvent() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	if(handler_->Instance==I2C1)
	{
		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
		PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			PANIC("Clock configuration for I2C1 failed");

		__HAL_RCC_GPIOG_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		    PG13     ------> I2C1_SDA
		    PG14     ------> I2C1_SCL
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();

		if (mode_ == MODE_IRQ) {
			/* I2C1 interrupt Init */
			HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
			HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
		}

	} else if(handler_->Instance==I2C2)	{
		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
		PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			PANIC("Clock configuration for I2C2 failed");

		__HAL_RCC_GPIOF_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		    PF0     ------> I2C2_SDA
		    PF1     ------> I2C2_SCL
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();

		if (mode_ == MODE_IRQ) {
			/* I2C2 interrupt Init */
			HAL_NVIC_SetPriority(I2C2_EV_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
			HAL_NVIC_SetPriority(I2C2_ER_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
		}
	}
	else if(handler_->Instance==I2C3) {
		/** Initializes the peripherals clock
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
		PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK3;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
			PANIC("Clock configuration for I2C3 failed");

		__HAL_RCC_GPIOC_CLK_ENABLE();
		/**I2C3 GPIO Configuration
		    PC0     ------> I2C3_SCL
		    PC1     ------> I2C3_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C3_CLK_ENABLE();

		if (mode_ == MODE_IRQ) {
			/* I2C3 interrupt Init */
			HAL_NVIC_SetPriority(I2C3_EV_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
			HAL_NVIC_SetPriority(I2C3_ER_IRQn, 2, 0);
			HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
		}
	}
}

void I2cDriver::onHWDeInitEvent() {
	if (handler_->Instance==I2C1) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		    PG13     ------> I2C1_SDA
		    PG14     ------> I2C1_SCL
		 */
		HAL_GPIO_DeInit(GPIOG, GPIO_PIN_13);
		HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14);

		if (mode_ == MODE_IRQ) {
			/* I2C1 interrupt DeInit */
			HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
			HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
		}
	}  else if(handler_->Instance==I2C2) {
		/* USER CODE BEGIN I2C2_MspDeInit 0 */

		/* USER CODE END I2C2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C2_CLK_DISABLE();

		/**I2C2 GPIO Configuration
		    PF0     ------> I2C2_SDA
		    PF1     ------> I2C2_SCL
		 */
		HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);
		HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);

		if (mode_ == MODE_IRQ) {
			/* I2C2 interrupt DeInit */
			HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
			HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
		}
	} else if(handler_->Instance==I2C3) {
		/* Peripheral clock disable */
		__HAL_RCC_I2C3_CLK_DISABLE();

		/**I2C3 GPIO Configuration
		    PC0     ------> I2C3_SCL
		    PC1     ------> I2C3_SDA
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);

		if (mode_ == MODE_IRQ) {
			/* I2C3 interrupt DeInit */
			HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
			HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
		}
	}
}

/**
 * @brief: dsdqdff
 * dfsfd
 * fsf
 * @ warning: La fonction N'EST PAS REENTRANTE !!! Attention à ne pas l'appeler dans plusieurs taches différentes
 */
bool I2cDriver::write(uint16_t addr, uint16_t length,uint8_t *data) {
	bool status = false;

	TickType_t timeoutFreertos = portMAX_DELAY;

	if (mode_ != MODE_POLLING) {
		if (timeout_!=portMAX_DELAY)
			timeoutFreertos = pdMS_TO_TICKS(timeout_);

		/*
		 * Sur un transfert I2C, on est toujours synchrone.
		 * On attend donc que le transfert (write) soit fini avant de sortir de la fonction
		 *
		 * D'où le semaphore APRES la fonction d'ecriture et pas avant comme dans le driver UART
		 */

		if (HAL_I2C_Master_Transmit_IT(handler_, addr, data, length) == HAL_OK) {
			if (xSemaphoreTake(txCompleteSemaphore_, timeoutFreertos) == pdTRUE)
				status = true;
		} else
			status = false;
	} else
		if (HAL_I2C_Master_Transmit(handler_, addr, data, length, timeout_) == HAL_OK)
			status=true;

	return status;
}

/**
 * @brief: dsdqdff
 * dfsfd
 * fsf
 * @ warning: La fonction N'EST PAS REENTRANTE !!! Attention à ne pas l'appeler dans plusieurs taches différentes
 */
bool I2cDriver::read(uint16_t addr, uint16_t length, uint8_t *data){
	bool status = false;
	TickType_t timeoutFreertos = portMAX_DELAY;

	if (mode_ != MODE_POLLING) {
		if (timeout_!=portMAX_DELAY)
			timeoutFreertos = pdMS_TO_TICKS(timeout_);

		if (HAL_I2C_Master_Receive_IT(handler_, addr, data, length) == HAL_OK) {
			if (xSemaphoreTake(rxCompleteSemaphore_,timeoutFreertos) == pdTRUE)
				status = true;
		}
	} else { // mode == MODE_POLLING
		if (HAL_I2C_Master_Receive(handler_, addr, data, length, timeout_) == HAL_OK)
			status = true;
	}

	return status;
}

/**
 * @brief: dsdqdff
 * dfsfd
 * fsf
 * @ warning: La fonction N'EST PAS REENTRANTE !!! Attention à ne pas l'appeler dans plusieurs taches différentes
 */
bool I2cDriver::regWrite(uint16_t addr,
		uint16_t reg_addr,
		uint16_t length,
		uint8_t *data) {
	bool status = false;

	TickType_t timeoutFreertos = portMAX_DELAY;

	if (mode_ != MODE_POLLING) {
		if (timeout_!=portMAX_DELAY)
			timeoutFreertos = pdMS_TO_TICKS(timeout_);

		/*
		 * Sur un transfert I2C, on est toujours synchrone.
		 * On attend donc que le transfert (write) soit fini avant de sortir de la fonction
		 *
		 * D'où le semaphore APRES la fonction d'ecriture et pas avant comme dans le driver UART
		 */

		if (HAL_I2C_Mem_Write_IT(handler_,
				addr,
				reg_addr,
				I2C_MEMADD_SIZE_8BIT,
				data,
				length) == HAL_OK) {
			if (xSemaphoreTake(txCompleteSemaphore_, timeoutFreertos) == pdTRUE)
				status = true;
		} else
			status = false;
	} else
		if (HAL_I2C_Mem_Write(handler_,
				addr,
				reg_addr,
				I2C_MEMADD_SIZE_8BIT,
				data,
				length,
				timeout_) == HAL_OK)
			status=true;

	return status;
}

/**
 * @brief: dsdqdff
 * dfsfd
 * fsf
 * @ warning: La fonction N'EST PAS REENTRANTE !!! Attention à ne pas l'appeler dans plusieurs taches différentes
 */
bool I2cDriver::regRead(uint16_t addr,
		uint16_t reg_addr,
		uint16_t length,
		uint8_t *data){
	bool status = false;
	TickType_t timeoutFreertos = portMAX_DELAY;

	if (mode_ != MODE_POLLING) {
		if (timeout_!=portMAX_DELAY)
			timeoutFreertos = pdMS_TO_TICKS(timeout_);

		if (HAL_I2C_Mem_Read_IT(handler_,
				addr,
				reg_addr,
				I2C_MEMADD_SIZE_8BIT,
				data,
				length) == HAL_OK) {
			if (xSemaphoreTake(rxCompleteSemaphore_,timeoutFreertos) == pdTRUE)
				status = true;
		}
	} else { // mode == MODE_POLLING
		if (HAL_I2C_Mem_Read(handler_,
				addr,
				reg_addr,
				I2C_MEMADD_SIZE_8BIT,
				data,
				length,
				timeout_) == HAL_OK)
			status = true;
	}

	return status;
}

/*
 * Ces fonctions sont appelées en fait sous interruption, sauf
 * onRXEvent(UART_RX_TIMER)
 */
void I2cDriver::onTXEvent() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Liberation du semaphore TX */
	xSemaphoreGiveFromISR(txCompleteSemaphore_, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The
	 actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void I2cDriver::onRXEvent() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Liberation du semaphore RX */
	xSemaphoreGiveFromISR(rxCompleteSemaphore_, &xHigherPriorityTaskWoken);

	/* Yield if xHigherPriorityTaskWoken is true. The  actual macro used here is port specific. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void I2cDriver::onErrorEvent() {
	assert_param(false);
}

// Callbacks statiques pour HAL
void I2cDriver::txCompleteCallback(I2C_HandleTypeDef *handler) {
	// Conversion unsafe, mais sous contrôle
	I2cDriver *instance = reinterpret_cast<I2cDriver*>(handler->MspInitCallback);
	if (instance) {
		instance->onTXEvent();
	}
}

void I2cDriver::rxCompleteCallback(I2C_HandleTypeDef *handler) {
	// Conversion unsafe, mais sous contrôle
	I2cDriver *instance = reinterpret_cast<I2cDriver*>(handler->MspInitCallback);
	if (instance) {
		instance->onRXEvent();
	}
}

void I2cDriver::errorCallback(I2C_HandleTypeDef *handler) {
	// Conversion unsafe, mais sous contrôle
	I2cDriver *instance = reinterpret_cast<I2cDriver*>(handler->MspInitCallback);
	if (instance) {
		instance->onErrorEvent();
	}
}

void I2cDriver::hwInit(I2C_HandleTypeDef *handler) {
	// Conversion unsafe, mais sous contrôle
	I2cDriver *instance = reinterpret_cast<I2cDriver*>(handler->MspDeInitCallback);
	if (instance) {
		instance->onHWInitEvent();
	}
}

void I2cDriver::hwDeInit(I2C_HandleTypeDef *handler) {
	// Conversion unsafe, mais sous contrôle
	I2cDriver *instance = reinterpret_cast<I2cDriver*>(handler->MspInitCallback);
	if (instance) {
		instance->onHWDeInitEvent();
	}
}
