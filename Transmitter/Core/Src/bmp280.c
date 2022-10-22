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

bool bmp280_write_reg8(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint8_t* reg_contents){
	if(HAL_I2C_Mem_Write(bmp->i2c_handle_, bmp->address_, reg_address, 1, reg_contents, 1, HAL_MAX_DELAY) == HAL_OK){
		return 1;
	}
	else{
		return 0;
	}
}

bool bmp280_get_compensation_data(BMP280_HandleTypedef* bmp){
	if(bmp280_read_reg16(bmp, 0x88, &(bmp->compensation_params_.dig_T1)) &&
	   bmp280_read_reg16(bmp, 0x8A, (uint16_t*) &(bmp->compensation_params_.dig_T2)) &&
	   bmp280_read_reg16(bmp, 0x8C, (uint16_t*) &(bmp->compensation_params_.dig_T3)) &&
	   bmp280_read_reg16(bmp, 0x8E, &(bmp->compensation_params_.dig_P1)) &&
	   bmp280_read_reg16(bmp, 0x90, (uint16_t*) &(bmp->compensation_params_.dig_P2)) &&
	   bmp280_read_reg16(bmp, 0x92, (uint16_t*) &(bmp->compensation_params_.dig_P3)) &&
	   bmp280_read_reg16(bmp, 0x94, (uint16_t*) &(bmp->compensation_params_.dig_P4)) &&
	   bmp280_read_reg16(bmp, 0x96, (uint16_t*) &(bmp->compensation_params_.dig_P5)) &&
	   bmp280_read_reg16(bmp, 0x98, (uint16_t*) &(bmp->compensation_params_.dig_P6)) &&
	   bmp280_read_reg16(bmp, 0x9A, (uint16_t*) &(bmp->compensation_params_.dig_P7)) &&
	   bmp280_read_reg16(bmp, 0x9C, (uint16_t*) &(bmp->compensation_params_.dig_P8)) &&
	   bmp280_read_reg16(bmp, 0x9E, (uint16_t*) &(bmp->compensation_params_.dig_P9))){
		return 1;
	}
	else{
		return 0;
	}
}

bool bmp280_init_force_mode(BMP280_HandleTypedef* bmp){
	bmp->address_ = BMP280_ADDRESS_0 << 1;	// SDO connected to ground
	bmp->mode_ = BMP280_SLEEP_MODE;     // to read in force mode it needs to be in sleep first
	bmp->filter_ = BMP280_FILTER_OFF;	// according to documentation for weather monitoring
	bmp->temperature_oversampling_ = BMP280_oversampling_x1;	// according to documentation
	bmp->pressure_oversampling_ = BMP280_oversampling_x1;		// according to documentation
	bmp->time_standby_ = BMP280_tsb_1000;	/* not relevant in forced mode, but dont want leave
											 * uninitialized
											 */

	// soft reset sensor
	if(!bmp280_write_reg8(bmp, BMP280_REG_ADDR_RESET, (uint8_t*) BMP280_RESET_VALUE)){
		return 0;
	}

	// wait for NVM data to copy
	while(1){
		uint8_t im_update;
		if(bmp280_read_reg8(bmp, BMP280_REG_ADDR_STATUS, &im_update) && (im_update & 1) == 0){
			break;
		}
	}

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

	// get compensation data
	if(!bmp280_get_compensation_data(bmp)){
		return 0;
	}

	// set config and control registers and write them
	uint8_t config_reg_settings = (bmp->time_standby_ << 5) | (bmp->filter_ << 2);
	if(!bmp280_write_reg8(bmp, BMP280_REG_ADDR_CONFIG, &config_reg_settings)){
		return 0;
	}

	uint8_t control_reg_settings = (bmp->temperature_oversampling_ << 5) | (bmp->pressure_oversampling_ << 2) | bmp->mode_;
	if(!bmp280_write_reg8(bmp, BMP280_REG_ADDR_CONTROL, &control_reg_settings)){
		return 0;
	}
	return 1;
}

bool bmp280_has_measurement_ended(BMP280_HandleTypedef* bmp){
	uint8_t measuring;
	if(bmp280_read_reg8(bmp, BMP280_REG_ADDR_STATUS, &measuring) && (measuring & 3) == 0){
		return 1;
	}
	else{
		return 0;
	}

}

bool bmp280_force_measurement(BMP280_HandleTypedef* bmp){
	//check if previous measurement has ended
	if(!bmp280_has_measurement_ended(bmp)){
		return 0;
	}

	//read previous control settings
	uint8_t control;
	if(!bmp280_read_reg8(bmp, BMP280_REG_ADDR_CONTROL, &control)){
		return 0;
	}

	//clears mode bits
	control &= ~0b11;

	//sets mode bits to FORCED
	control |= BMP280_FORCED_MODE;

	if(!bmp280_write_reg8(bmp, BMP280_REG_ADDR_CONTROL, &control)){
		return 0;
	}

	return 1;
}

int32_t t_fine;		// global variable as defined in datasheet (used later in pressure measurement)
int32_t bmp280_compensate_T_int32(BMP280_HandleTypedef* bmp, int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)bmp->compensation_params_.dig_T1<<1))) * ((int32_t)bmp->compensation_params_.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)bmp->compensation_params_.dig_T1)) * ((adc_T>>4) - ((int32_t)bmp->compensation_params_.dig_T1))) >> 12) *
	((int32_t)bmp->compensation_params_.dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bmp280_compensate_P_int64(BMP280_HandleTypedef* bmp, int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bmp->compensation_params_.dig_P6;
	var2 = var2 + ((var1*(int64_t)bmp->compensation_params_.dig_P5)<<17);
	var2 = var2 + (((int64_t)bmp->compensation_params_.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp->compensation_params_.dig_P3)>>8) + ((var1 * (int64_t)bmp->compensation_params_.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp->compensation_params_.dig_P1)>>33;

	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bmp->compensation_params_.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)bmp->compensation_params_.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->compensation_params_.dig_P7)<<4);
	return (uint32_t)p;
}

bool bmp280_get_measurements(BMP280_HandleTypedef* bmp,
			uint32_t* pressure, int32_t* temperature){
	//check if measurement has ended
	if(!bmp280_has_measurement_ended(bmp)){
		return 0;
	}

	uint8_t raw_data[6];
	uint32_t raw_pressure;
	int32_t raw_temperature;

	//read data in burst
	if(HAL_I2C_Mem_Read(bmp->i2c_handle_, bmp->address_, BMP280_REG_ADDR_PRESS, 1, raw_data, 6, HAL_MAX_DELAY) != HAL_OK){
		return 0;
	}

	raw_pressure = raw_data[0] << 12 | raw_data[1] << 4 | raw_data[3] >> 4;
	raw_temperature = raw_data[3] << 12 | raw_data[4] << 4 | raw_data[5] >> 4;

	*pressure = bmp280_compensate_P_int64(bmp, raw_pressure);
	*temperature = bmp280_compensate_T_int32(bmp, raw_temperature);
	return 1;
}
