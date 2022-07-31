/*
 * am2320.h
 *
 *  Created on: Jul 22, 2022
 *      Author: Jasiek
 */

#ifndef INC_AM2320_H_
#define INC_AM2320_H_

#include "main.h"
#include <stdio.h>

// real address is 0xB8, but i2c has some weird stuff with left shifting it
#define AM2320_ADDRESS 0x5C

// define names for error codes (1 = prompt for sensor measurement failed, 2 = response from sensor failed)
#define SEND_ERROR 1
#define READ_ERROR 2
#define CRC_ERROR 3

typedef struct{
	I2C_HandleTypeDef* i2c_handle_;
	uint8_t sensor_address_;
	uint8_t sensor_data_[8];
	int16_t last_temperature;
	uint16_t last_humidity;
} AM2320_HandleTypeDef;

AM2320_HandleTypeDef am2320_init(I2C_HandleTypeDef* i2c_handle, uint8_t sensor_address);

void am2320_error_handler(uint8_t error_code);

uint16_t am2320_crc_checker(uint8_t* data, uint8_t size);

void am2320_read_temperature_and_humidity(AM2320_HandleTypeDef* am2320);

// currently not used
void am2320_write(AM2320_HandleTypeDef* am2320, uint8_t reg_addr);

#endif /* INC_AM2320_H_ */
