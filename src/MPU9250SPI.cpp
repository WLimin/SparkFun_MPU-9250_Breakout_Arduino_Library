/*
 * MPU9250SPI.cpp
 *
 *  Created on: 2018年8月12日
 *      Author: wang
 */
#include <MPU9250SPI.h>
#include <Arduino.h>
#include <stdint.h>

MPU9250SPI::MPU9250SPI(int8_t csPin, SPIClass &spiInterface, uint32_t spi_freq) {
	// Use hardware SPI communication
	// If used with sparkfun breakout board
	// https://www.sparkfun.com/products/13762 , change the pre-soldered JP2 to
	// enable SPI (solder middle and left instead of middle and right) pads are
	// very small and re-soldering can be very tricky. I2C highly recommended.

	_csPin = csPin;
	_spi = &spiInterface;

	_interfaceSpeed = spi_freq;
}
// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
void MPU9250SPI::kickHardware(void) {
	_spi->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
	_spi->transfer(0x00); // Send null byte
	_spi->endTransaction();
}
// Select slave IC by asserting CS pin
void MPU9250SPI::select(void) {
	digitalWrite(_csPin, LOW);
}

// Select slave IC by deasserting CS pin
void MPU9250SPI::deselect(void) {
	digitalWrite(_csPin, HIGH);
}

uint8_t MPU9250SPI::readBytesSPI(uint8_t registerAddress, uint8_t count, uint8_t* dest) {
	_spi->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
	select();

	_spi->transfer(registerAddress | READ_FLAG);

	uint8_t i;

	for (i = 0; i < count; i++) {
		dest[i] = _spi->transfer(0x00);
// #ifdef SERIAL_DEBUG
//     Serial.print("readBytesSPI::Read byte: 0x");
//     Serial.println(dest[i], HEX);
// #endif
	}

	_spi->endTransaction();
	deselect();

	delayMicroseconds(50);

	return i; // Return number of bytes written

	/*
	 #ifdef SERIAL_DEBUG
	 Serial.print("MPU9250::writeByteSPI slave returned: 0x");
	 Serial.println(returnVal, HEX);
	 #endif
	 return returnVal;
	 */

	/*
	 // Set slave address of AK8963 and set AK8963 for read
	 writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG);

	 Serial.print("\nBHW::I2C_SLV0_ADDR set to: 0x");
	 Serial.println(readByte(_I2Caddr, I2C_SLV0_ADDR), HEX);

	 // Set address to start read from
	 writeByteSPI(I2C_SLV0_REG, registerAddress);
	 // Read bytes from magnetometer
	 //
	 Serial.print("\nBHW::I2C_SLV0_CTRL gets 0x");
	 Serial.println(READ_FLAG | count, HEX);

	 // Read count bytes from registerAddress via I2C_SLV0
	 Serial.print("BHW::readBytesSPI: return value test: ");
	 Serial.println(writeByteSPI(I2C_SLV0_CTRL, READ_FLAG | count));
	 */
}

// Read a byte from the given register address using SPI
uint8_t MPU9250SPI::readByteSPI(uint8_t registerAddress) {
	return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}
uint8_t MPU9250SPI::readMagByteSPI(uint8_t registerAddress) {
	setupMagForSPI();

	writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));
	writeByteSPI(50, registerAddress);
	writeByteSPI(52, 0b11000000);        // Command the read into I2C_SLV4_DI register, cause an interrupt when complete

	// Wait for the data to be ready
	uint8_t I2C_MASTER_STATUS = readByteSPI(54);

	uint32_t count = 0;
	while (((I2C_MASTER_STATUS & 0x40) == 0) && (count++ < 100000)) { // Checks against the I2C_SLV4_DONE bit in the I2C master status register
		I2C_MASTER_STATUS = readByteSPI(54);
	}
	if (count > 10000) {
		Serial.println(F("Timed out"));
	}

	return readByteSPI(53);   // Read the data that is in the SLV4_DI register
}

uint8_t MPU9250SPI::writeMagByteSPI(uint8_t registerAddress, uint8_t data) {
	setupMagForSPI();

	writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));
	writeByteSPI(50, registerAddress);
	writeByteSPI(51, data);
	writeByteSPI(52, 0b11000000);        // Command the read into I2C_SLV4_DI register, cause an interrupt when complete

	uint8_t I2C_MASTER_STATUS = readByteSPI(54);
	uint32_t count = 0;
	while (((I2C_MASTER_STATUS & 0b01000000) == 0) && (count++ < 10000)) { // Checks against the I2C_SLV4_DONE bit in the I2C master status register
		I2C_MASTER_STATUS = readByteSPI(54);
	}
	if (count > 10000) {
		Serial.println(F("Timed out"));
	}
	return 0x00;
}
uint8_t MPU9250SPI::writeByteSPI(uint8_t registerAddress, uint8_t writeData) {
	uint8_t returnVal;

	_spi->beginTransaction(SPISettings(_interfaceSpeed, MSBFIRST, SPI_MODE));
	select();

	_spi->transfer(registerAddress);
	returnVal = _spi->transfer(writeData);

	deselect();
	_spi->endTransaction();
// #ifdef SERIAL_DEBUG
//   Serial.print("MPU9250::writeByteSPI slave returned: 0x");
//   Serial.println(returnVal, HEX);
// #endif
	return returnVal;
}
// Wire.h read and write protocols
uint8_t MPU9250SPI::writeByte(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data) {
	(void) deviceAddress;
	return writeByteSPI(registerAddress, data);
}

// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t MPU9250SPI::readByte(uint8_t deviceAddress, uint8_t registerAddress) {
	if (deviceAddress == AK8963_ADDRESS) {
		return readMagByteSPI(registerAddress);
	} else {
		return readByteSPI(registerAddress);
	}
}

uint8_t MPU9250SPI::readBytes(uint8_t deviceAddress, uint8_t registerAddress, uint8_t count, uint8_t* dest) {
	(void) deviceAddress;
	return readBytesSPI(registerAddress, count, dest);
}

uint8_t MPU9250SPI::readWhoAmI_AK8963(void) {
//	return ak8963WhoAmI_SPI();
	return MPU9250::readWhoAmI_AK8963();
}

void MPU9250SPI::setupMagForSPI(void) {
	// Use slave 4 for talking to the magnetometer
	writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));    // Set the SLV_4_ADDR register to the magnetometer's address
	writeByteSPI(52, 0b00000000);                  // Setup SLV_4 control as needed (but not set to do an operation yet)

	writeByteSPI(36, 0b10000000);   // Enable the multi-master mode
}
void MPU9250SPI::initAK8963(float* destination) {
	MPU9250::initAK8963(destination);
	setupMagForSPI();
}
void MPU9250SPI::initMPU9250(void) {
	MPU9250::initMPU9250();
	setupMagForSPI();
}
bool MPU9250SPI::begin(void) {
	_spi->begin();
	pinMode(_csPin, OUTPUT);
	deselect();
	kickHardware();
	return magInit();
}
// Read the WHOAMI (WIA) register of the AK8963
// TODO: This method has side effects
uint8_t MPU9250SPI::ak8963WhoAmI_SPI(void) {
	uint8_t response, oldSlaveAddress, oldSlaveRegister, oldSlaveConfig;
	// Save state
	oldSlaveAddress = readByteSPI(I2C_SLV0_ADDR);
	oldSlaveRegister = readByteSPI(I2C_SLV0_REG);
	oldSlaveConfig = readByteSPI(I2C_SLV0_CTRL);
#ifdef SERIAL_DEBUG
	Serial.print("Old slave address: 0x");
	Serial.println(oldSlaveAddress, HEX);
	Serial.print("Old slave register: 0x");
	Serial.println(oldSlaveRegister, HEX);
	Serial.print("Old slave config: 0x");
	Serial.println(oldSlaveConfig, HEX);
#endif

	// Set the I2C slave addres of AK8963 and set for read
	response = writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG);
	// I2C slave 0 register address from where to begin data transfer
	response = writeByteSPI(I2C_SLV0_REG, 0x00);
	// Enable 1-byte reads on slave 0
	response = writeByteSPI(I2C_SLV0_CTRL, 0x81);
	delayMicroseconds(1);
	// Read WIA register
	response = writeByteSPI(WHO_AM_I_AK8963 | READ_FLAG, 0x00);

	// Restore state
	writeByteSPI(I2C_SLV0_ADDR, oldSlaveAddress);
	writeByteSPI(I2C_SLV0_REG, oldSlaveRegister);
	writeByteSPI(I2C_SLV0_CTRL, oldSlaveConfig);

	return response;
}
bool MPU9250SPI::magInit(void) {
	// Reset registers to defaults, bit auto clears
	writeByteSPI(0x6B, 0x80);
	// Auto select the best available clock source
	writeByteSPI(0x6B, 0x01);
	// Enable X,Y, & Z axes of accel and gyro
	writeByteSPI(0x6C, 0x00);
	// Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
	writeByteSPI(0x1A, 0x01);
	// Self tests off, gyro set to +/-2000 dps FS
	writeByteSPI(0x1B, 0x18);
	// Self test off, accel set to +/- 8g FS
	writeByteSPI(0x1C, 0x08);
	// Bypass DLPF and set accel bandwidth to 184 Hz
	writeByteSPI(0x1D, 0x09);
	// Configure INT pin (active high / push-pull / latch until read)
	writeByteSPI(0x37, 0x30);
	// Enable I2C master mode
	// TODO Why not do this 11-100 ms after power up?
	writeByteSPI(0x6A, 0x20);
	// Disable multi-master and set I2C master clock to 400 kHz
	//https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls says
	// enabled multi-master... TODO Find out why
	writeByteSPI(0x24, 0x0D);
	// Set to write to slave address 0x0C
	writeByteSPI(0x25, 0x0C);
	// Point save 0 register at AK8963's control 2 (soft reset) register
	writeByteSPI(0x26, 0x0B);
	// Send 0x01 to AK8963 via slave 0 to trigger a soft restart
	writeByteSPI(0x63, 0x01);
	// Enable simple 1-byte I2C reads from slave 0
	writeByteSPI(0x27, 0x81);
	// Point save 0 register at AK8963's control 1 (mode) register
	writeByteSPI(0x26, 0x0A);
	// 16-bit continuous measurement mode 1
	writeByteSPI(0x63, 0x12);
	// Enable simple 1-byte I2C reads from slave 0
	writeByteSPI(0x27, 0x81);

	// TODO: Remove this code
	uint8_t ret = ak8963WhoAmI_SPI();
#ifdef SERIAL_DEBUG
	Serial.print("MPU9250::magInit to return ");
	Serial.println((ret == 0x48) ? "true" : "false");
#endif
	return ret == 0x48;
}

