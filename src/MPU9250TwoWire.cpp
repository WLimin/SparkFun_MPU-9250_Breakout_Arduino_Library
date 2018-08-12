/*
 * MPU9250TwoWire.cpp
 *
 *  Created on: 2018年8月12日
 *      Author: wang
 */

#include <MPU9250TwoWire.h>
MPU9250TwoWire::MPU9250TwoWire(uint8_t address, TwoWire &wirePort, uint32_t clock_frequency) {
	_I2Caddr = address;
	_wire = &wirePort;
	_interfaceSpeed = clock_frequency;
}

uint8_t MPU9250TwoWire::writeByteWire(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
	_wire->setClock(_interfaceSpeed);           // Reset to the desired speed, in case other devices required a slowdown
	_wire->beginTransmission(deviceAddress);    // Initialize the Tx buffer
	_wire->write(registerAddress);              // Put slave register address in Tx buffer
	_wire->write(data);                         // Put data in Tx buffer
	_wire->endTransmission();                   // Send the Tx buffer
	// TODO: Fix this to return something meaningful
	// return NULL; // In the meantime fix it to return the right type
	return 0;
}

// Read a byte from the given register address from device using I2C
uint8_t MPU9250TwoWire::readByteWire(uint8_t deviceAddress, uint8_t registerAddress) {
	uint8_t data; // `data` will store the register data

	// Initialize the Tx buffer
	_wire->beginTransmission(deviceAddress);
	// Put slave register address in Tx buffer
	_wire->write(registerAddress);
	// Send the Tx buffer, but send a restart to keep connection alive
	_wire->endTransmission(false);
	// Read one byte from slave register address
	_wire->requestFrom(deviceAddress, (uint8_t) 1);
	// Fill Rx buffer with result
	data = _wire->read();
	// Return data read from slave register
	return data;
}

// Read 1 or more bytes from given register and device using I2C
uint8_t MPU9250TwoWire::readBytesWire(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest) {
	// Initialize the Tx buffer
	_wire->beginTransmission(deviceAddress);
	// Put slave register address in Tx buffer
	_wire->write(registerAddress);
	// Send the Tx buffer, but send a restart to keep connection alive
	_wire->endTransmission(false);

	uint8_t i = 0;
	// Read bytes from slave register address
	_wire->requestFrom(deviceAddress, count);
	while (_wire->available()) {
		// Put read results in the Rx buffer
		dest[i++] = _wire->read();
	}

	return i; // Return number of bytes written
}
// Wire.h read and write protocols
uint8_t MPU9250TwoWire::writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
	return writeByteWire(deviceAddress, registerAddress, data);
}

// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t MPU9250TwoWire::readByte(uint8_t deviceAddress, uint8_t registerAddress) {
	return readByteWire(deviceAddress, registerAddress);
}

uint8_t MPU9250TwoWire::readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest) {
	return readBytesWire(deviceAddress, registerAddress, count, dest);
}
bool MPU9250TwoWire::begin() {
	_wire->begin();
	_wire->setClock(_interfaceSpeed);
	return true;
}
