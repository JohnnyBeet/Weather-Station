/*
 * am2320.h
 *
 *  Created on: Jul 22, 2022
 *      Author: Jasiek
 */

#ifndef INC_AM2320_H_
#define INC_AM2320_H_

#include "main.h"

#define AM2320_ADDRESS 0xB8

typedef struct{
	I2C_HandleTypeDef* i2c_handle_;
	uint8_t sensor_address_;
	uint8_t sensor_data_[8];
} AM2320_HandleTypeDef;

AM2320_HandleTypeDef am2320_init(I2C_HandleTypeDef* i2c_handle, uint8_t sensor_address);

void am2320_error_handler();

void am2320_read(AM2320_HandleTypeDef* am2320);

// currently not used
void am2320_write(AM2320_HandleTypeDef* am2320, uint8_t reg_addr);

void am2320_get_temperature_and_humidity(AM2320_HandleTypeDef* am2320);

#endif /* INC_AM2320_H_ */
