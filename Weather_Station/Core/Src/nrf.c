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
	uint8_t address_rx = 0;
	// set csn low to initiate spi transmission
	NRF_CSN_SET_LOW;
	// send read command
	if(!NRF_SPI_RW(NRF_CMD_R_REGISTER | address, &address_rx)){
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
	uint8_t address_rx = 0;
	// set csn low to initiate spi transmission
	NRF_CSN_SET_LOW;
	// send write command
	if(!NRF_SPI_RW(NRF_CMD_W_REGISTER | address, &address_rx)){
		return NRF_ERROR;
	}
	// command is locked by csn, so it will write bytes until length is 0
	while(length--){
		if(!NRF_SPI_RW(*data, &address_rx)){
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
 * @brief sets air data rate, power amplifier for tx, and lna for rx
 * @param[in] rate : data rate
 * @param[in] amp : power of the transmitter [dBm]
 * @param[in] lna : lna off or on
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_RadioParams(NRF_AirDataRate rate, NRF_PowerAmplifier amp, NRF_LNAsetup lna){
	uint8_t reg;
	if(!NRF_ReadRegs(NRF_REG_RF_SETUP, &reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~(NRF_MASK_RF_DR  | NRF_MASK_RF_PWR | NRF_MASK_LNA);		// sets bits to 0
	reg |= ((rate << 3) | (amp << 1) | lna);
	if(!NRF_WriteRegs(NRF_REG_RF_SETUP, &reg, 1)){
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
	if(!NRF_WriteRegs(NRF_REG_RF_CH, &safe_freq, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}
/*
 * @brief sets mode to rx or tx
 * @param[in] mode : some enum
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_Mode(NRF_Mode mode){
	uint8_t reg;
	if(!NRF_ReadRegs(NRF_REG_CONFIG, &reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_MODE;		// sets bit to 0
	reg |= mode;
	if(!NRF_WriteRegs(NRF_REG_CONFIG, &reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets mode to rx or tx
 * @param[in] mode : some enum
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_DynamicPayload(NRF_DynamicPayload dpl){
	uint8_t reg;
	if(!NRF_ReadRegs(NRF_REG_FEATURE, &reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_EN_DPL;		// sets bit to 0
	reg |= dpl;
	if(!NRF_WriteRegs(NRF_REG_FEATURE, &reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets crc configuration
 * @param[in] crc : on or off
 * @param[in] bytes: number of crc bytes
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_CRC(NRF_CRC crc, NRF_CRCbytes bytes){
	uint8_t reg;
	if(!NRF_ReadRegs(NRF_REG_CONFIG, &reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_CRC;		// sets bits to 0
	reg |= ((crc << 3) | (bytes << 2));
	if(!NRF_WriteRegs(NRF_REG_RF_SETUP, &reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets address width
 * @param[in] crc : on or off
 * @param[in] bytes: number of crc bytes
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_PipeAddressWidth(NRF_AddressWidth width){
	uint8_t safe_width = width & NRF_MASK_AW;
	if(!NRF_WriteRegs(NRF_REG_SETUP_AW, &safe_width, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets retransmission parameters
 * @param[in] ard : delay between retx
 * @param[in] arc : how many retransmissions allowed
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_Retransmission(NRF_RetransmitDelay ard, NRF_RetransmitCount arc){
	uint8_t retransmit_data = (ard << 4) | arc;
	if(!NRF_WriteRegs(NRF_REG_SETUP_RETR, &retransmit_data, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief sets address for given data pipe
 * @param[in] pipe : number of pipe (used for base address of pipes)
 * @param[in] address : pointer to array with address (LSB first, its pretty crucial)
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_PipeAddress(NRF_Pipe pipe, uint8_t* address){
	uint8_t address_width;
	if(!NRF_ReadRegs(NRF_REG_SETUP_AW, &address_width, 1)){
		return NRF_ERROR;
	}

	switch(pipe){
		case RX_PIPE_0:
		case RX_PIPE_1:
		case TX_PIPE:
			// for pipe 0-1 send all three bytes
			if(!NRF_WriteRegs(NRF_REG_RX_ADDR_BASE + pipe, address, address_width)){
				return NRF_ERROR;
			}
			break;
		case RX_PIPE_2:
		case RX_PIPE_3:
		case RX_PIPE_4:
		case RX_PIPE_5:
			// for pipe 2-5 and tx send only LSB
			if(!NRF_WriteRegs(NRF_REG_RX_ADDR_BASE + pipe, address, 1)){
				return NRF_ERROR;
			}
			break;
		default:
			// some different value, return error
			return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief configuration for given data pipe. RX only, on TX side setting address is all
 * @param[in] pipe : number of pipe (used for base address of pipes)
 * @param[in] auto_ack : enable enhanced shockburst for given pipe
 * @param[in]] payload_length : length (in bytes) of data packets send through pipe
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */

bool NRF_SET_PipeRX(NRF_Pipe pipe, NRF_AutoAcknowledge auto_ack, uint8_t payload_length){
	// enable pipe
	uint8_t pipe_en;
	if(!NRF_ReadRegs(NRF_REG_EN_RXADDR, &pipe_en, 1)){
		return NRF_ERROR;
	}
	pipe_en &= ~(NRF_MASK_EN_BASE << pipe);
	pipe_en |= 1 << pipe;
	if(!NRF_WriteRegs(NRF_REG_EN_RXADDR, &pipe_en, 1)){
		return NRF_ERROR;
	}

	// set auto acknowledge
	uint8_t aa_en;
	if(!NRF_ReadRegs(NRF_REG_EN_AA, &aa_en, 1)){
		return NRF_ERROR;
	}
	aa_en &= ~(NRF_MASK_EN_BASE << pipe);
	aa_en |= 1 << pipe;
	if(!NRF_WriteRegs(NRF_REG_EN_AA, &aa_en, 1)){
		return NRF_ERROR;
	}

	// set payload length (used when dpl is disabled)
	uint8_t safe_pl_len = NRF_MASK_RX_PW_P & payload_length;
	if(!NRF_WriteRegs(NRF_REG_EN_AA, &safe_pl_len, 1)){
		return NRF_ERROR;
	}

	// if everything passed return ok
	return NRF_OK;

}

/*
 * @brief flush rx fifo
 *
 * @return nothing
 * @retval 1 if successfully flushed, 0 if something went wrong
 */

bool NRF_FlushRXFifo(void){
	uint8_t rx_buff = 0;
	if(!NRF_SPI_RW(NRF_CMD_FLUSH_RX, &rx_buff)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief flush tx fifo
 *
 * @return nothing
 * @retval 1 if successfully flushed, 0 if something went wrong
 */

bool NRF_FlushTXFifo(void){
	uint8_t rx_buff = 0;
	NRF_CSN_SET_LOW;
	if(!NRF_SPI_RW(NRF_CMD_FLUSH_TX, &rx_buff)){
		return NRF_ERROR;
	}
	NRF_CSN_SET_HIGH;
	return NRF_OK;
}

/*
 * @brief write tx fifo
 * @param[in] data : buffer with data to send
 * @param[in] length : data length
 *
 * @return nothing
 * @retval 1 if successfully written, 0 if something went wrong
 */

bool NRF_ReadWritePayload(uint8_t* data, uint8_t length){
	uint8_t rx_buff = 0;
	NRF_CSN_SET_LOW;
	if(!NRF_SPI_RW(NRF_CMD_W_TX_PAYLOAD, &rx_buff)){
		return NRF_ERROR;
	}
	while(length--){
		if(!NRF_SPI_RW(*data, rx_buff)){
			return NRF_ERROR;
		}
		data++;
	}
	NRF_CSN_SET_HIGH;
	return NRF_OK;
}

/*
 * @brief read rx fifo
 * @param[in] data : buffer for data to read
 * @param[in] length : data length
 *
 * @return nothing
 * @retval 1 if successfully read, 0 if something went wrong
 */


bool NRF_ReadRxPayload(uint8_t* data, uint8_t length){
	uint8_t rx_buff = 0;
	NRF_CSN_SET_LOW;
	if(!NRF_SPI_RW(NRF_CMD_R_RX_PAYLOAD, &rx_buff)){
		return NRF_ERROR;
	}
	while(length--){
		if(!NRF_SPI_RW(NOP, data)){
			return NRF_ERROR;
		}
		data++;
	}
	NRF_CSN_SET_HIGH;
	return NRF_OK;
}

/*
 * @brief clear irq bits in status register
 *
 * @return nothing
 * @retval 1 if successfully cleared, 0 if something went wrong
 */

bool NRF_ClearIRQFlags(void){
	uint8_t status_reg;
	if(!NRF_ReadRegs(NRF_REG_STATUS, &status_reg, 1)){
		return NRF_ERROR;
	}

	// write 1 to clear bits
	status_reg |= (NRF_MASK_RX_DR | NRF_MASK_TX_DS | NRF_MASK_MAX_RT);
	if(!NRF_WriteRegs(NRF_REG_STATUS, &status_reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief set flag to signalize interrupt occurence
 * @param[in] nrfInterruptFlag : pointer to global flag indicating interrupt state
 * @return nothing
 * @retval nothing
 */

void NRF_IRQ_Event(uint8_t* nrfInterruptFlag){
	*nrfInterrupt = 1;
}

/*
 * @brief reads interrupt source and
 * @param[in] nrfInterruptFlag : pointer to global flag indicating interrupt state
 *
 * @return nothing
 * @retval nothing
 */

bool NRF_IRQ_Handler(uint8_t* nrfInterrupt){

}
