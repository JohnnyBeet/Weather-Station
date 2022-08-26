/*
 * bmp20.c
 *
 *  Created on: 24 sie 2022
 *      Author: jasie
 */

#include "bmp280.h"

#define BMP280_REG_ADDR_ID 0xD0
#define BMP280_REG_ADDR_RESET 0xE0
#define BMP280_REG_ADDR_STATUS 0xF3
#define BMP280_REG_ADDR_CONTROL 0xF4
#define BMP280_REG_ADDR_CONFIG 0xF5
#define BMP280_REG_ADDR_TEMP 0xFA
#define BMP280_REG_ADDR_PRESS 0xF7
#define BMP280_ID 0x58
#define BMP280_RESET_VALUE 0x86

/*
 * TODO: make two functions out of this: one for reading
 */
bool bmp280_read_reg16(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint16_t* reg_contents){
	uint8_t reg_temp[2];
	if(HAL_I2C_Mem_Read(bmp->i2c_handle_, bmp->address_, reg_address, 1, reg_temp, 2, HAL_MAX_DELAY) == HAL_OK){
		*reg_contents = (reg_temp[1]<<8) | reg_temp[0];
		return 1;
	}
	else{
		return 0;
	}
}

bool bmp280_read_reg8(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint8_t* reg_contents){
	uint8_t reg_temp;
	if(HAL_I2C_Mem_Read(bmp->i2c_handle_, bmp->address_, reg_address, 1, &reg_temp, 1, HAL_MAX_DELAY) == HAL_OK){
		*reg_contents = reg_temp;
		return 1;
	}
	else{
		return 0;
	}
}



bool bmp280_get_compensation_data(BMP280_HandleTypedef* bmp){
	if(bmp280_read_reg16(bmp, 0x88, &(bmp->compenstation_params_.dig_T1)) &&
	   bmp280_read_reg16(bmp, 0x8A, (uint16_t*) &(bmp->compenstation_params_.dig_T2)) &&
	   bmp280_read_reg16(bmp, 0x8C, (uint16_t*) &(bmp->compenstation_params_.dig_T3)) &&
	   bmp280_read_reg16(bmp, 0x8E, &(bmp->compenstation_params_.dig_P1)) &&
	   bmp280_read_reg16(bmp, 0x90, (uint16_t*) &(bmp->compenstation_params_.dig_P2)) &&
	   bmp280_read_reg16(bmp, 0x92, (uint16_t*) &(bmp->compenstation_params_.dig_P3)) &&
	   bmp280_read_reg16(bmp, 0x94, (uint16_t*) &(bmp->compenstation_params_.dig_P4)) &&
	   bmp280_read_reg16(bmp, 0x96, (uint16_t*) &(bmp->compenstation_params_.dig_P5)) &&
	   bmp280_read_reg16(bmp, 0x98, (uint16_t*) &(bmp->compenstation_params_.dig_P6)) &&
	   bmp280_read_reg16(bmp, 0x9A, (uint16_t*) &(bmp->compenstation_params_.dig_P7)) &&
	   bmp280_read_reg16(bmp, 0x9C, (uint16_t*) &(bmp->compenstation_params_.dig_P8)) &&
	   bmp280_read_reg16(bmp, 0x9E, (uint16_t*) &(bmp->compenstation_params_.dig_P9))){
		return 1;
	}
	else{
		return 0;
	}
}

void bmp280_init_force_mode(BMP280_HandleTypedef* bmp){
	bmp->address_ = BMP280_ADDRESS_0;	// SDO connected to ground
	bmp->mode_ = BMP280_FORCED_MODE;     // to read in force mode it needs to be in sleep first
	bmp->filter_ = BMP280_FILTER_OFF;	// according to documentation for weather monitoring
	bmp->temperature_oversampling_ = BMP280_oversampling_x1;	// according to documentation
	bmp->pressure_oversampling_ = BMP280_oversampling_x1;		// according to documentation
	bmp->time_standby_ = BMP280_tsb_1000;	/* not relevant in forced mode, but dont want leave
											 * uninitialized
											 */
}

bool bmp280_force_measurement(BMP280_HandleTypedef* bmp){

	// check if address is proper
	if(bmp->address_ != BMP280_ADDRESS_0 && bmp->address_ != BMP280_ADDRESS_1){
		return 0;
	}

	// read and check if proper id
	uint8_t read_id = 0;
	if(!bmp280_read_reg8(bmp, BMP280_REG_ADDR_ID, &read_id)){
		return 0;
	}
	else if(read_id != BMP280_ID){
		return 0;
	}

	// soft reset sensor

	uint8_t config_reg_settings = (bmp->temperature_oversampling_ << 3) | (bmp->pressure_oversampling_ << 3) | bmp->mode_;
//	uint8_t control_reg_settings = (bmp)
}
