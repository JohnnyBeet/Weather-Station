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

	// left shift cause i2c is weird
	am2320_.sensor_address_ = sensor_address << 1;
	am2320_.last_temperature = 0;
	am2320_.last_humidity = 0;
	return am2320_;
}

// taken from sensor documentation
uint16_t am2320_crc_checker(uint8_t* data, uint8_t size){

	uint16_t crc =0xFFFF;
	uint8_t i;
	while(size--)
	{
		crc ^=*data++;
		for(i=0;i<8;i++)
		{
			if(crc & 0x01)
			{
				crc>>=1;
				crc^=0xA001;
			}
			else
			{
				crc>>=1;
			}
		}
	}
	return crc;
}

void am2320_read_temperature_and_humidity(AM2320_HandleTypeDef* am2320){
	/* regs:
	 * empty_reg for waking the sensor up
	 * read_regs for triggering measurements and reading from sensor registers
	 * */
	uint8_t empty_reg[1] = { 0x00 };
	uint8_t read_regs[3] = { 0x03, 0x00, 0x04 };

	/* wake sensor:
	 * sends address (empty reg and 0 size for not writing anything)
	 * returns HAL_ERROR cause sensor doesn't ACK waking up
	 * after sending address it should wait for <800us;3ms>,
	 * but either hardware or HAL library for i2c seems to manage this
	 * */
	HAL_I2C_Master_Transmit(am2320->i2c_handle_, am2320->sensor_address_, empty_reg, 0, 1000);

	/*TODO: comment properly
	 * prompts for measurement
	 * */
	if(HAL_I2C_Master_Transmit(am2320->i2c_handle_, am2320->sensor_address_, read_regs, 3, 1000) != HAL_OK){
		printf("Prompting for measurement went wrong!\n");
		//TODO: this is tragic, need to rework this!!!
		HAL_I2C_Init(am2320->i2c_handle_);
		return;
	}

	/*TODO: comment properly
	 * receives am2320->sensor_data_
	 * */
	if(HAL_I2C_Master_Receive(am2320->i2c_handle_, am2320->sensor_address_, am2320->sensor_data_, 8, 1000) != HAL_OK){
		printf("Receiving am2320->sensor_data_ failed!\n");
		//TODO: this is tragic, need to rework this!!!
		HAL_I2C_Init(am2320->i2c_handle_);
		return;
	}

	// check crc
	uint16_t sensor_crc = ((am2320->sensor_data_[7]<<8)+am2320->sensor_data_[6]);
	uint16_t calculated_crc = am2320_crc_checker((am2320->sensor_data_), 6);
	if(sensor_crc != calculated_crc){
		printf("Wrong CRC!\n");
		return;
	}

	// different types, because temperature may be negative
	am2320->last_temperature = (int16_t)((am2320->sensor_data_[4]<<8)+am2320->sensor_data_[5]);
	am2320->last_humidity = ((am2320->sensor_data_[2]<<8)+am2320->sensor_data_[3]);


}
