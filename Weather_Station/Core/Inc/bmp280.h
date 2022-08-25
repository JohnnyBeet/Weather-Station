/*
 * bmp280.h
 *
 *  Created on: 24 sie 2022
 *      Author: jasie
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

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
	uint8_t dig_T2;
	uint8_t dig_T3;
	uint16_t dig_P1;
	uint8_t digP2;
	uint8_t digP3;
	uint8_t digP4;
	uint8_t digP5;
	uint8_t digP6;
	uint8_t digP7;
	uint8_t digP8;
	uint8_t digP9;
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
	BMP280_CompensationParams compenstation_params_;
	BMP280_Filter filter_;
	BMP280_Oversampling oversampling_;
	BMP280_TimeStandby time_standby_;
}BMP280_HandleTypedef;


/*
 * @brief initialize sensor instance for working in forced mode.
 * @param[in] bmp : pointer to bmp sensor structure.
 * @param[out] bmp : initialized bmp instance.
 * 					 If fields fail to initalize then there will be rubbish in them
 *
 * @return None
 */

void bmp280_init_force_mode(BMP280_HandleTypedef* bmp);

/*
 * @brief reads compensation parameters from sensor and puts them in sensor handle
 * @param[in] sensor_comp_params : pointer to struct of compensation params
 * @param[out] sensor_comp_params : initialized compensation params in sensor handle
 *
 * @return None
 */

void bmp280_get_compensation_data(BMP280_CompensationParams* sensor_comp_params);

#endif /* INC_BMP280_H_ */
