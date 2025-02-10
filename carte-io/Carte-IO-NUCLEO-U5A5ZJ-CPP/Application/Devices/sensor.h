/*
 * sensor.h
 *
 *  Created on: Feb 10, 2025
 *      Author: dimercur
 */

#ifndef DEVICES_SENSOR_H_
#define DEVICES_SENSOR_H_

class Sensor {
public:
	Sensor() {}
	virtual ~Sensor() {}

	static bool probe(void* handler) {return false;}
};

#endif /* DEVICES_SENSOR_H_ */
