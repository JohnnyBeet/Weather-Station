/*
 * nrf.c
 *
 *  Created on: 5 paź 2022
 *      Author: jasie
 */


#include "nrf/nrf.h"
#include "nrf/nrf_defines.h"

/*
 * @brief Reads from given register (multiple bytes)
 * @param[in] spi : handle to spi
 * @param[in] address : address of the register to read
 * @param[out] data : pointer to data, where register settings will be written
 * @param[in] length : how many bytes read from register
 *
 * @return data from given register
 * @retval 1 in case of successful read , 0 in case of failure
 */

// TODO: rewrite it with low level HAL TransmitReceive
bool NRF_ReadRegs(SPI_HandleTypeDef* spi, uint8_t address, uint8_t* data, uint8_t length){
	// buffer for receiving
	uint8_t* address_rx;
	// set csn low to initiate spi transmission
	NRF_CSN_SET_LOW;
	// give read command (passed as argument)
	if(HAL_SPI_TransmitReceive(spi, &address, address_rx, 1, 1000) != HAL_OK){
		return 0;
	}
	// command is locked by csn, so it will read bytes until length is 0
	while(length--){
		if(HAL_SPI_TransmitReceive(spi, pTxData, pRxData, Size, Timeout))
	}


}

/*
 * @brief Writes to given register (1 byte)
 * @param[in] address : address of the register to write
 *
 * @return None
 * @retval 1 in case of successful write, 0 in case of failure
 */

bool NRF_write_reg(uint8_t address, uint8_t data);
