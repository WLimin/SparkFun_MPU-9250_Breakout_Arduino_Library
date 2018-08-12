/*
 * MPU9250SPI.h
 *
 *  Created on: 2018年8月12日
 *      Author: wang
 */

#ifndef MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250SPI_H_
#define MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250SPI_H_
#include <stdint.h>
#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>

//#define READ_FLAG 0x80
//#define NOT_SPI -1
#define SPI_DATA_RATE 1000000 // 1MHz is the max speed of the MPU-9250
#define SPI_MODE SPI_MODE3

class MPU9250SPI: public MPU9250 {
public:
	MPU9250SPI(int8_t csPin, SPIClass &spiInterface = SPI, uint32_t spi_freq = SPI_DATA_RATE);
	virtual ~MPU9250SPI(){}
	void initMPU9250(void);
	void initAK8963(float* destination);
	void kickHardware(void);
    bool begin(void);
    uint8_t readWhoAmI_AK8963(void);
    bool magInit(void);

	SPIClass * _spi;							// Allows for use of different SPI ports
	int8_t _csPin; 							// SPI chip select pin
	uint32_t _interfaceSpeed;				// Stores the desired I2C or SPi clock rate

protected:
	uint8_t writeByteSPI(uint8_t registerAddress, uint8_t writeData);
	uint8_t writeMagByteSPI(uint8_t subAddress, uint8_t data);
	uint8_t readByteSPI(uint8_t subAddress);
	uint8_t readBytesSPI(uint8_t registerAddress, uint8_t count, uint8_t* dest);

	uint8_t readMagByteSPI(uint8_t subAddress);
	void select();
	void deselect();
private:
	uint8_t writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data);
	uint8_t readByte(uint8_t deviceAddress, uint8_t registerAddress);
	uint8_t readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest);
	void setupMagForSPI();
	uint8_t ak8963WhoAmI_SPI();
};

#endif /* MPU_9250_BREAKOUT_ARDUINO_LIBRARY_SRC_MPU9250SPI_H_ */
