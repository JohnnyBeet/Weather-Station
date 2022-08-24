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

/*
 * real address is 0xB8, but i2c has some weird stuff with left shifting it
 */
#define AM2320_ADDRESS 0x5C

/*
 * define names for error codes (1 = prompt for sensor measurement failed, 2 = response from sensor failed)
 */
#define SEND_ERROR 1
#define READ_ERROR 2
#define CRC_ERROR 3

/*
 * handle for the sensor
 * some parameters:
 * 	   sensor_data_ : raw data read from sensor
 *     last_humidity, last_temperature : last correct measurements after conversion
 * TODO: think about holding the raw data as pointer to some static/global table
 * 		 might delete last_ variables, dont know if holding last read is sensible
 */
typedef struct{
	I2C_HandleTypeDef* i2c_handle_;
	uint8_t sensor_address_;
	uint8_t sensor_data_[8];
	int16_t last_temperature;
	uint16_t last_humidity;
} AM2320_HandleTypeDef;

/*
 * @brief initalize AM2320 sensor handle
 * @param[in] I2C_HandleTypeDef : pointer to I2C handle for sensor data transfer
 * @param[in] sensor_address : address of the sensor to be used in i2c communication
 * 
 * @return AM2320 handle with all initialized fields (those not passed to function are zero)
 * @retval AM2320_HandleTypeDef if successfull, some garbage otherwise
 */
AM2320_HandleTypeDef am2320_init(I2C_HandleTypeDef* i2c_handle, uint8_t sensor_address);

/*
 * @brief handle error in am2320 communication; currently not used
 */
void am2320_error_handler(uint8_t error_code);

/*
 * @brief checks, whether sensor crc and computed crc are the same (validates data)
 * 
 */
uint16_t am2320_crc_checker(uint8_t* data, uint8_t size);

void am2320_read_temperature_and_humidity(AM2320_HandleTypeDef* am2320);

// currently not used
void am2320_write(AM2320_HandleTypeDef* am2320, uint8_t reg_addr);

#endif /* INC_AM2320_H_ */
