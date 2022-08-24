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

#define BMP_280_ADDRESS_0 0x76
#define BMP_280_ADDRESS_1 0x77



#endif /* INC_BMP280_H_ */
