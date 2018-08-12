/*
 * MPU9250TwoWire.h
 *
 *  Created on: 2018年8月12日
 *      Author: wang
 */

#ifndef MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250TWOWIRE_H_
#define MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250TWOWIRE_H_
#include <stdint.h>
#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>
// Using the MPU-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
// The previous preprocessor directives were sensitive to the location that the user defined AD1
// Now simply define MPU9250_ADDRESS as one of the two following depending on your application
//#define MPU9250_ADDRESS_AD1 0x69  // Device address when ADO = 1
//#define MPU9250_ADDRESS_AD0 0x68  // Device address when ADO = 0
//#define AK8963_ADDRESS  0x0C   // Address of magnetometer

class MPU9250TwoWire: public MPU9250 {
public:
	MPU9250TwoWire(uint8_t address = MPU9250_ADDRESS_AD0, TwoWire &wirePort = Wire, uint32_t clock_frequency = 100000);
	virtual ~MPU9250TwoWire(){}
    bool begin(void);

	TwoWire * _wire;						// Allows for use of various I2C ports
	uint32_t _interfaceSpeed;				// Stores the desired I2C or SPi clock rate
private:
	uint8_t writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);
	uint8_t readByte(uint8_t deviceAddress, uint8_t registerAddress);
	uint8_t readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest);

	uint8_t writeByteWire(uint8_t, uint8_t, uint8_t);
	uint8_t readByteWire(uint8_t address, uint8_t subAddress);
	uint8_t readBytesWire(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest);

};

#endif /* MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250TWOWIRE_H_ */
