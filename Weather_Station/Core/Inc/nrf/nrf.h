/*
 * nrf.h
 *
 *  Created on: 30 wrz 2022
 *      Author: jasie
 */

#ifndef INC_NRF_NRF_H_
#define INC_NRF_NRF_H_

#include "../main.h"
#include "nrf_defines.h"
#include <stdbool.h>

/*
 * Macros for hardware pin setting
 * CSN: chip select, used in spi communication (every transmission has to start with high to low transition)
 * CE: used for switching between rx and tx modes
 */

#define NRF_CSN_SET_HIGH	HAL_GPIO_WritePin(SPI2_CSN_GPIO_Port, SPI2_CSN_Pin, GPIO_PIN_SET)
#define NRF_CSN_SET_LOW		HAL_GPIO_WritePin(SPI2_CSN_GPIO_Port, SPI2_CSN_Pin, GPIO_PIN_RESET)
#define NRF_CE_SET_HIGH		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define NRF_CE_SET_LOW		HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)

/*
 * @brief Reads from given register (1 byte)
 * @param[in] address : address of the register to read
 *
 * @return data from given register
 * @retval 1 in case of successful read , 0 in case of failure
 */

bool NRF_read_reg(uint8_t address, uint8_t* data);

/*
 * @brief Writes to given register (1 byte)
 * @param[in] address : address of the register to write
 *
 * @return None
 * @retval 1 in case of successful write, 0 in case of failure
 */

bool NRF_write_reg(uint8_t address, uint8_t data);


#endif /* INC_NRF_NRF_H_ */
