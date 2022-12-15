/*
 * bmp280.h
 *
 *  Created on: 24 sie 2022
 *  Edited by: JohnnyBeet
 *

 * Ciastkolog.pl (https://github.com/ciastkolog)
 * 
*/
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 sheinz (https://github.com/sheinz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include <stdbool.h>
#include <stdint.h>
#include <stm32f4xx_hal.h>

/*
 * taken from documentation:
 * connecting SDO to the ground results in address 0x76
 * connecting SDO to Vdd results in address 0x77
 * SDO cannot be left floating, because then address is unidentified!
 */
typedef enum{
	BMP280_ADDRESS_0 = 0x76,
	BMP280_ADDRESS_1 = 0x77
}BMP280_Address;

/*
 * enum for different sensor operation modes
 */
typedef enum{
	BMP280_SLEEP_MODE = 0,
	BMP280_FORCED_MODE = 1,
	BMP280_NORMAL_MODE = 2
}BMP280_Mode;

/*
 * Struct representing compensation parameters.
 * Used as a container.
 * names taken from documentation
 * TODO: check if they change during sensor being active
 */
typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
}BMP280_CompensationParams;

/*
 * Enum for holding time standby values. Only used for normal mode.
 * Naming convention : BMP280_tsb_x_y where x is time in ms, and y is decimal part of the time
 */
typedef enum{
	BMP280_tsb_0_5 = 0,
	BMP280_tsb_62_5 = 1,
	BMP280_tsb_125 = 2,
	BMP280_tsb_250 = 3,
	BMP280_tsb_500 = 4,
	BMP280_tsb_1000 = 5,
	BMP280_tsb_2000 = 6,
	BMP280_tsb_4000 = 7,
}BMP280_TimeStandby;

/*
 * Enum for iir filter coefficients.
 */
typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

/*
 * Enum for oversampling (it's the same for both temperature and pressure)
 */
typedef enum{
	BMP280_skipped = 0,
	BMP280_oversampling_x1 = 1,
	BMP280_oversampling_x2 = 2,
	BMP280_oversampling_x4 = 3,
	BMP280_oversampling_x8 = 4,
	BMP280_oversampling_x16 = 5
}BMP280_Oversampling;

/*
 * BMP280 handle, used to store configuration
 */
typedef struct{
	I2C_HandleTypeDef* i2c_handle_;
	BMP280_Address address_;
	BMP280_Mode mode_;
	BMP280_CompensationParams compensation_params_;
	BMP280_Filter filter_;
	BMP280_Oversampling temperature_oversampling_;
	BMP280_Oversampling pressure_oversampling_;
	BMP280_TimeStandby time_standby_;
}BMP280_HandleTypedef;

/*
 * #TODO: comment
 */
bool bmp280_read_reg16(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint16_t* reg_contents);

/*
 * #TODO: comment
 */
bool bmp280_read_reg8(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint8_t* reg_contents);

/*
 * #TODO: comment
 */
bool bmp280_write_reg8(BMP280_HandleTypedef* bmp, uint8_t reg_address, uint8_t* reg_contents);

/*
 * @brief reads compensation parameters from sensor and puts them in sensor handle
 * @param[in] bmp280 : pointer to sensor handle
 *
 * @return None
 */
bool bmp280_get_compensation_data(BMP280_HandleTypedef* bmp);

/* TODO: fix
 * @brief initialize sensor instance for working in forced mode.
 * @param[in] bmp280 : pointer to sensor handle
 * @param[out] bmp : initialized bmp instance.
 * 					 If fields fail to initalize then there will be rubbish in them
 *
 * @return None
 */
bool bmp280_init_force_mode(BMP280_HandleTypedef* bmp);

/*
 * @brief checks if sensor has finished conversion.
 * @param[in] bmp280 : pointer to sensor handle
 *
 * @return 0 or 1
 * @retval 0 if measurement hasn't ended, 1 if measurement has ended
 */
bool bmp280_has_measurement_ended(BMP280_HandleTypedef* bmp);

/*
 * @brief initiates force measurement
 * @param[in] bmp280 : pointer to sensor handle
 *
 * @return None
 * @retval Doesn't return anything, other function used for extracting data from registers
 */
bool bmp280_force_measurement(BMP280_HandleTypedef* bmp);

/*
 * TODO
 */
int32_t bmp280_compensate_T_int32(BMP280_HandleTypedef* bmp, int32_t adc_T);

/*
 *
 */
uint32_t bmp280_compensate_P_int64(BMP280_HandleTypedef* bmp, int32_t adc_P);

/*
 * @brief reads register data and returns them raw (20bit integers -> 32bit ints)
 * @param[in] bmp280 : pointer to sensor handle
 * @param[in] raw_pressure : pointer to uint32 which will hold pressure value
 * @param[in] raw_temperature : pointer to int32 which will hold temperature value
 *
 * @return None
 * @retval Properly read values of temp & press or will call an error handler
 */
bool bmp280_get_measurements(BMP280_HandleTypedef* bmp280,
			uint32_t* raw_pressure, int32_t* raw_temperature);

/*
 * @brief compensates and calculates raw data into human-appealing format
 * param[in] raw_pressure : value of pressure read directly from the sensor
 * param[in] raw_temperature : value of temperature read directly from the sensor
 * param[out] pressure : pointer to which function will write computed pressure value
 * param[out] temperature : pointer to which function will write computed temperature value
 *
 * @return None
 * @retval Properly calculated values of temp & press or will call an error handler
 */


#endif /* INC_BMP280_H_ */
