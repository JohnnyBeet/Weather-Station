/*
 * nrf.h
 *
 *  Created on: 30 wrz 2022
 *      Author: jasie
 */

#ifndef INC_NRF_NRF_H_
#define INC_NRF_NRF_H_

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
 * @brief: low level spi transmit receive function. Hardcoded to use SPI2.
 * @param[in] transmit_buff : transmit command through SPI. Use NOP in case reading only
 *
 * @return: read value or nothing in case of writing only
 * @retval: uint8_t register value, nothing or calls error handler
 */
uint8_t NRF_SPI_RW(uint8_t transmit_buff, uint8_t* receive_buff);


/*
 * general purpose functions
 */

bool NRF_ReadRegs(uint8_t address, uint8_t* data, uint8_t length);
bool NRF_WriteRegs(uint8_t address, uint8_t* data, uint8_t length);
bool NRF_Init();

/*
 * setters
 */
bool NRF_SET_AirDataRate(NRF_AirDataRate rate);
bool NRF_SET_Frequency(NRF_Frequency freq);
bool NRF_SET_PowerAmplifier(NRF_PowerAmplifier amp);
bool NRF_SET_LNAsetup(NRF_LNAsetup lna);
bool NRF_SET_Mode(NRF_Mode mode);
bool NRF_SET_DynamicPayload(NRF_DynamicPayload dpl);
bool NRF_SET_CRC(NRF_CRC crc, NRF_CRCbytes);
bool NRF_SET_PipeAddressWidth(NRF_AddressWidth width);
bool NRF_SET_AutoAcknowledge(NRF_AutoAcknowledge ack);
bool NRF_SET_Retransmission(NRF_RetransmitDelay ard, NRF_RetransmitCount arc);
bool NRF_SET_PipeAddress(NRF_Pipe pipe, uint8_t* address);
bool NRF_SET_PipeRX(NRF_Pipe pipe, uint8_t auto_ack, uint8_t payload_length);
bool NRF_SET_IRQFlags();

/*
 * getters
 */

bool NRF_GET_RXFifoState();
bool NRF_GET_TXFifoState();
#endif /* INC_NRF_NRF_H_ */
