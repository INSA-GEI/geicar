/*
 * Gpio.h
 *
 *  Created on: Jan 6, 2025
 *      Author: dimercur
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32u5xx.h"

#include "FreeRTOS.h"

#include "sensor.h"
#include "messages.h"
#include "taskhandler.h"

class GpioMessage : public Message {
public:
	typedef struct {
		uint8_t pins;
		uint8_t vals;
	} GPIOPins_TypeDef;

	GpioMessage();
	GpioMessage(GPIOPins_TypeDef pins);

	void setPins (GPIOPins_TypeDef pins);
	GPIOPins_TypeDef getPins(void);

	GpioMessage* copy();
	std::string toString();

	/**
     * Comparison operator
     * @param msg Message to be compared
     * @return true if message are equal, false otherwise
     */
    bool operator==(const GpioMessage& msg) {
        return ((messageID_ == msg.messageID_) &&
        		(pins_.pins == msg.pins_.pins) &&
				(pins_.vals == msg.pins_.vals));
    }

    /**
     * Difference operator
     * @param msg Message to be compared
     * @return true if message are different, false otherwise
     */
    bool operator!=(const GpioMessage& msg) {
    	return !((messageID_ == msg.messageID_) &&
    	        		(pins_.pins == msg.pins_.pins) &&
    					(pins_.vals == msg.pins_.vals));
    }
protected:
    /**
     * Message pins value
     */
    GPIOPins_TypeDef pins_;

    /**
     * Verify if message ID is compatible with current message type
     * @param id Message ID
     * @return true, if message ID is acceptable, false otherwise
     */
    bool checkID(MessageID id);
};

class Gpio : public Sensor {
public:
	Gpio(const char* taskName);
	~Gpio();

	static GpioMessage* fromFrame(Sensor::RawData_Typedef raw);
	static Sensor::RawData_Typedef toFrame(GpioMessage* msg);
private:
	// Handler de la tâche
	TaskHandler taskHandler_;

	// Méthode de la classe appelée par la tâche
	void run(void);
};

#endif /* GPIO_H_ */
