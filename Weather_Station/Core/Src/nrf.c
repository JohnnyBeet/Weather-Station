/*
 * nrf.c
 *
 *  Created on: 5 paź 2022
 *      Author: jasie
 */


#include "nrf/nrf.h"

uint8_t NRF_SPI_RW(uint8_t transmit_buff, uint8_t* receive_buff){
	if(HAL_SPI_TransmitReceive(&hspi2, &transmit_buff, receive_buff, 1, 1000) != HAL_OK){
		return NRF_ERROR;
	}
	return NRF_OK;
}


/*
 * @brief Reads from given register (multiple bytes)
 * @param[in] address : address of the register
 * @param[out] data : pointer to data where we read to
 * @param[in] length : how many bytes read from register
 *
 * @return data from given register
 * @retval 1 in case of successful read , 0 in case of failure
 */

bool NRF_ReadRegs(uint8_t address, uint8_t* data, uint8_t length){
	// buffer for receiving
	uint8_t* address_rx;
	// set csn low to initiate spi transmission
	NRF_CSN_SET_LOW;
	// send read command
	if(!NRF_SPI_RW(NRF_CMD_R_REGISTER | address, address_rx)){
		return NRF_ERROR;
	}
	// command is locked by csn, so it will read bytes until length is 0
	while(length--){
		// passing NOP to not write anything
		if(!NRF_SPI_RW(NRF_CMD_NOP, data)){
			return NRF_ERROR;
		}
		++data;
	}
	// release spi
	NRF_CSN_SET_HIGH;
	return NRF_OK;

}

/*
 * @brief Writes to given register (1 byte)
 * @param[in] address : address of the register
 * @param[in] data : pointer to data where we write to
 * @param[in] length : how many bytes read from register
 *
 * @return None
 * @retval 1 in case of successful write, 0 in case of failure
 */

bool NRF_WriteRegs(uint8_t address, uint8_t* data, uint8_t length){
	// buffer for receiving
	uint8_t* address_rx;
	// set csn low to initiate spi transmission
	NRF_CSN_SET_LOW;
	// send write command
	if(!NRF_SPI_RW(NRF_CMD_W_REGISTER | address, address_rx)){
		return NRF_ERROR;
	}
	// command is locked by csn, so it will write bytes until length is 0
	while(length--){
		// passing NOP to not write anything
		if(!NRF_SPI_RW(*data, address_rx)){
			return NRF_ERROR;
		}
		++data;
	}
	// release spi
	NRF_CSN_SET_HIGH;
	return NRF_OK;
}

/*
 * @brief Init function. Defines basic configuration:
 * - only one pipe, enabled enhanced shockburst, up to 5 retransmits
 * - rest of parameters defined in function body
 * @param[in] nrf : pointer to transciever handler
 */
bool NRF_Init(NRF_HandleTypedef* nrf){
	// below parameters are configurable, wanted to keep them in one place
	nrf->rate_ = RATE_1Mbps;
	nrf->frequency_ = (uint8_t)10;	// channel frequency 2410MHz
	nrf->power_amp_ = dBm_0; 	// max power, TX only
	nrf->lna_ = LNA_HIGH;	// rx only
	nrf->dpl_ = DPL_OFF;
	nrf->crc_ = CRC_ENABLE;
	nrf->crc_bytes_ = CRCB_ONE;
	nrf->address_width_ = ADR_THREE;
	nrf->ack_ = AA_ON;
	nrf->retransmissions_ = ARC_TEN;
	nrf->ret_delay_ = DELAY_2000uS;
}

/*
 * @brief sets air data rate
 * @param[in] rate : enum type
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_AirDataRate(NRF_AirDataRate rate){
	uint8_t* reg;
	if(!NRF_ReadRegs(NRF_REG_RF_SETUP, reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_RF_DR;		// sets data rate bit to 0
	reg |= rate;
	if(!NRF_WriteRegs(NRF_REG_RF_SETUP, reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets frequency channel
 * @param[in] rate : uint8_t, range 0-125
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_Frequency(NRF_Frequency freq){
	uint8_t safe_freq = freq & NRF_MASK_RF_CH;
	if(!NRF_WriteRegs(NRF_REG_RF_CH, safe_freq, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets output power. TX MODE ONLY!
 * @param[in] amp : some enum
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_PowerAmplifier(NRF_PowerAmplifier amp){
	uint8_t* reg;
	if(!NRF_ReadRegs(NRF_REG_RF_SETUP, reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_RF_PWR;		// sets power bits to 0
	reg |= amp;
	if(!NRF_WriteRegs(NRF_REG_RF_SETUP, reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 *
 */

bool NRF_SET_LNAsetup(NRF_LNAsetup lna){

}


