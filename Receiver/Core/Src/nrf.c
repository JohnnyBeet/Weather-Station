/*
 * nrf.c
 *
 *  Created on: 5 paź 2022
 *      Author: jasie
 */


#include "nrf/nrf.h"

uint8_t NRF_SPI_RW(uint8_t transmit_buff, uint8_t* receive_buff){
	if(HAL_SPI_TransmitReceive(&hspi3, &transmit_buff, receive_buff, 1, 1000) != HAL_OK){
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
	nrf->retransmissions_ = ARC_TEN;
	nrf->ret_delay_ = DELAY_2000uS;

	if(!NRF_SET_RadioParams(nrf->rate_, nrf->power_amp_, nrf->lna_)){
		return NRF_ERROR;
	}
	if(!NRF_SET_Frequency(nrf->frequency_)){
		return NRF_ERROR;
	}
	if(!NRF_SET_DynamicPayload(nrf->dpl_)){
		return NRF_ERROR;
	}
	if(!NRF_SET_CRC(nrf->crc_, nrf->crc_bytes_)){
		return NRF_ERROR;
	}
//	uint8_t reg=0;
//		NRF_ReadRegs(NRF_REG_RF_SETUP, &reg, 1);
//		printf("RF_SETUP: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
//				(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	if(!NRF_SET_PipeAddressWidth(nrf->address_width_)){
		return NRF_ERROR;
	}
	if(!NRF_SET_Retransmission(nrf->ret_delay_,  nrf->retransmissions_)){
		return NRF_ERROR;
	}
	// flush fifos
	if(!NRF_FlushTXFifo()){
		return NRF_ERROR;
	}
	if(!NRF_FlushRXFifo()){
		return NRF_ERROR;
	}

	// clear irq flags
	if(!NRF_ClearIRQFlags()){
		return NRF_ERROR;
	}
	return NRF_OK;
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
 * @brief sets power mode to power up or power down
 * @param[in] mode : some enum
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */
bool NRF_SET_PowerMode(NRF_PowerMode pwr){
	uint8_t reg;
	if(!NRF_ReadRegs(NRF_REG_CONFIG, &reg, 1)){
		return NRF_ERROR;
	}
	reg &= ~NRF_MASK_PWR_MODE;		// sets bit to 0
	reg |= pwr << 1;
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
	if(!NRF_WriteRegs(NRF_REG_CONFIG, &reg, 1)){
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
			address_width += 2;
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
	if(!NRF_WriteRegs(NRF_REG_RX_PW_BASE+pipe, &safe_pl_len, 1)){
		return NRF_ERROR;
	}

	// if everything passed return ok
	return NRF_OK;
}


/*
 * @brief disable pipe and set aa for it in transmitter
 * @param[in] pipe : number of pipe (used for base address of pipes)
 * @param[in] auto_ack : enable enhanced shockburst for given pipe
 *
 * @return nothing
 * @retval 1 if successfully set, 0 if something went wrong
 */
bool NRF_SET_PipeTX(NRF_Pipe pipe, NRF_AutoAcknowledge auto_ack){
//	// disable pipe
//	uint8_t pipe_en;
//	if(!NRF_ReadRegs(NRF_REG_EN_RXADDR, &pipe_en, 1)){
//		return NRF_ERROR;
//	}
//	pipe_en &= ~(NRF_MASK_EN_BASE << pipe);
//	if(!NRF_WriteRegs(NRF_REG_EN_RXADDR, &pipe_en, 1)){
//		return NRF_ERROR;
//	}
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

	// if everything passed return ok
	return NRF_OK;
}

/*
 * @brief get number of rx pipe ready to read from
 * @param[in] pipe : pointer to pipe variable
 *
 * @return nothing
 * @retval 1 if successfully get, 0 if something went wrong
 */
bool NRF_GET_PipeNumber(uint8_t* pipe){
	uint8_t status_reg = 0;
	if(!NRF_ReadRegs(NRF_REG_STATUS, &status_reg, 1)){
		return NRF_ERROR;
	}
	*pipe = (status_reg & NRF_MASK_RX_P_NO) >> 1;
	return NRF_OK;
}

/*
 * @brief get length of payload for given pipe
 * @param[in] pipe : pipe number
 *
 * @return nothing
 * @retval 1 if successfully get, 0 if something went wrong
 */
bool NRF_GET_PayloadLength(uint8_t pipe, uint8_t* length){
	if(!NRF_ReadRegs(NRF_REG_RX_PW_BASE+pipe, length, 1)){
		return NRF_ERROR;
	}
	*length &= NRF_MASK_RX_PW_P;
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
	NRF_CSN_SET_LOW;
	if(!NRF_SPI_RW(NRF_CMD_FLUSH_RX, &rx_buff)){
		return NRF_ERROR;
	}
	NRF_CSN_SET_HIGH;
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
bool NRF_WriteTxPayload(uint8_t* data, uint8_t length){
	uint8_t rx_buff = 0;
	NRF_CSN_SET_LOW;
	if(!NRF_SPI_RW(NRF_CMD_W_TX_PAYLOAD, &rx_buff)){
		return NRF_ERROR;
	}
	while(length--){
		if(!NRF_SPI_RW(*data, &rx_buff)){
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
		if(!NRF_SPI_RW(NRF_CMD_NOP, data)){
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
 * @brief reads interrupt source and starts appropriate handler
 * @param[in] nrfInterruptFlag : pointer to global flag indicating interrupt state
 *
 * @return nothing
 * @retval nothing
 */
bool NRF_IRQ_Callback(uint8_t* nrfInterrupt, uint8_t* data_buffer){

	if(*nrfInterrupt){

		uint8_t status_reg = 0;

		if(!NRF_ReadRegs(NRF_REG_STATUS, &status_reg, 1)){
			return NRF_ERROR;
		}
		// if flag is high, run one of handlers
		if(NRF_MASK_RX_DR & status_reg){
			// interrupt: data ready rx fifo
			if(!NRF_IRQ_RxHandler(data_buffer)){
				return NRF_ERROR;
			}
		}
		else if(NRF_MASK_TX_DS & status_reg){
			// interrupt: data sent tx fifo
			if(!NRF_IRQ_TxHandler()){
				return NRF_ERROR;
			}
		}
		else if(NRF_MASK_MAX_RT & status_reg){
			// interrupt : max no of retransmissions
			if(!NRF_IRQ_MaxHandler()){
				return NRF_ERROR;
			}
		}
		*nrfInterrupt = 0;
		return NRF_OK;
	}
	else{
		// somehow this was called without flag raised, so call error
		return NRF_ERROR;
	}

}

/*
 * @brief handle rx interrupt
 * @param[in] data_buffer : pointer to buffer to read data into
 *
 * @return nothing
 * @retval nothing
 */
bool NRF_IRQ_RxHandler(uint8_t* data_buffer){
	//get pipe number
	uint8_t pipe = 0x07; //sets as empty rx
	if(!NRF_GET_PipeNumber(&pipe)){
		return NRF_ERROR;
	}

	//get length
	uint8_t length = 0;
	if(!NRF_GET_PayloadLength(pipe, &length)){
		return NRF_ERROR;
	}

	//read data
	if(!NRF_ReadRxPayload(data_buffer, length)){
		return NRF_ERROR;
	}

	// deassert IRQ pin; my code supports
	// only one interrupt at a time, so I can clear all flags
	if(!NRF_ClearIRQFlags()){
		return NRF_ERROR;
	}
	if(!NRF_FlushRXFifo()){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief handle tx interrupt
 *
 * @return nothing
 * @retval nothing
 */
bool NRF_IRQ_TxHandler(void){
	// this only deasserts irq flags
	if(!NRF_ClearIRQFlags()){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * @brief handle max interrupt
 *
 * @return nothing
 * @retval nothing
 */
bool NRF_IRQ_MaxHandler(void){
	// clear fifo and deassert pin
	// this config simply discards packet
	if(!NRF_FlushTXFifo()){
		return NRF_ERROR;
	}

	if(!NRF_ClearIRQFlags()){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * comment properly
 */
bool NRF_ResetPlos(){
	uint8_t rf_ch_reg = 0;
	if(!NRF_ReadRegs(NRF_REG_RF_CH, &rf_ch_reg, 1)){
		return NRF_ERROR;
	}
	if(!NRF_WriteRegs(NRF_REG_RF_CH, &rf_ch_reg, 1)){
		return NRF_ERROR;
	}
	return NRF_OK;
}

/*
 * debug dump fuction, prints registers in readable format
 */

void NRF_PrintConfig(){
	uint8_t reg = 0;
	NRF_ReadRegs(NRF_REG_CONFIG, &reg, 1);
	printf("CONFIG: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_EN_AA, &reg, 1);
	printf("EN_AA: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_EN_RXADDR, &reg, 1);
	printf("EN_RXADDR: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_SETUP_AW, &reg, 1);
	printf("SETUP_AW: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_SETUP_RETR, &reg, 1);
	printf("SETUP_RETR: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_RF_CH, &reg, 1);
	printf("RF_CH: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_RF_SETUP, &reg, 1);
	printf("RF_SETUP: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_STATUS, &reg, 1);
	printf("STATUS: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_OBSERVE_TX, &reg, 1);
	printf("OBSERVE_TX: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	uint8_t address[6];
	for(int i=0; i<6; i++){
		NRF_ReadRegs(NRF_REG_RX_ADDR_BASE+i, address, 6);
		printf("RX_ADDR_P%d: 0x%X%X%X%X%X%X\n", i, address[0], address[1], address[2], address[3], address[4], address[5]);
	}
	NRF_ReadRegs(NRF_REG_TX_ADDR, address, 6);
	printf("RX_ADDR_TX: 0x%X%X%X%X%X%X\n", address[0], address[1], address[2], address[3], address[4], address[5]);

	for(int i=0; i<6; i++){
		NRF_ReadRegs(NRF_REG_RX_PW_BASE+i, &reg, 1);
		printf("RX_PW_P%d: %d%d%d%d%d%d%d%d\n",i, (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
				(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);
	}
	NRF_ReadRegs(NRF_REG_FIFO_STATUS, &reg, 1);
	printf("FIFO_STATUS: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_DYNPD, &reg, 1);
	printf("DYNPD: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);

	NRF_ReadRegs(NRF_REG_FEATURE, &reg, 1);
	printf("FEATURE: %d%d%d%d%d%d%d%d\n", (reg >> 7) & 1,(reg >> 6) & 1,(reg >> 5) & 1,
			(reg >> 4) & 1,(reg >> 3) & 1,(reg >> 2) & 1,(reg >> 1) & 1, reg & 1);
}
