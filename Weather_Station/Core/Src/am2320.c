/*
 * am2320.c
 *
 *  Created on: Jul 23, 2022
 *      Author: Jasiek
 */


#include "am2320.h"

AM2320_HandleTypeDef am2320_init(I2C_HandleTypeDef* i2c_handle, uint8_t sensor_address){
	AM2320_HandleTypeDef am2320_;
	am2320_.i2c_handle_ = i2c_handle;
	am2320_.sensor_address_ = sensor_address;
	return am2320_;
}

void am2320_error_handler(){

}

void am2320_read(AM2320_HandleTypeDef* am2320){
	if(HAL_I2C_Mem_Read(am2320->i2c_handle_, (uint16_t)am2320->sensor_address_, (uint16_t)0x00, 1, am2320->sensor_data_, sizeof(am2320->sensor_data_), HAL_MAX_DELAY) != HAL_OK){
		am2320_error_handler();
	}
}

void am2320_get_temperature_and_humidity(AM2320_HandleTypeDef* am2320){

}
