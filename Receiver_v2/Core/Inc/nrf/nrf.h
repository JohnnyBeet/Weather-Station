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

#define NRF_CSN_SET_HIGH	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET)
#define NRF_CSN_SET_LOW		HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET)
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
bool NRF_FlushRXFifo(void);
bool NRF_FlushTXFifo(void);
bool NRF_WriteTxPayload(uint8_t* data, uint8_t length);
bool NRF_ReadRxPayload(uint8_t* data, uint8_t length);
bool NRF_ClearIRQFlags(void);
bool NRF_Init();
bool NRF_ResetPlos();

/*
 * setters
 */
bool NRF_SET_RadioParams(NRF_AirDataRate rate, NRF_PowerAmplifier amp, NRF_LNAsetup lna);
bool NRF_SET_Frequency(NRF_Frequency freq);
bool NRF_SET_LNAsetup(NRF_LNAsetup lna);
bool NRF_SET_Mode(NRF_Mode mode);
bool NRF_SET_PowerMode(NRF_PowerMode pwr);
bool NRF_SET_DynamicPayload(NRF_DynamicPayload dpl);
bool NRF_SET_CRC(NRF_CRC crc, NRF_CRCbytes bytes);
bool NRF_SET_PipeAddressWidth(NRF_AddressWidth width);
//bool NRF_SET_AutoAcknowledge(NRF_AutoAcknowledge ack); prolly wont be used
bool NRF_SET_Retransmission(NRF_RetransmitDelay ard, NRF_RetransmitCount arc);
bool NRF_SET_PipeAddress(NRF_Pipe pipe, uint8_t* address);
bool NRF_SET_PipeRX(NRF_Pipe pipe, uint8_t auto_ack, uint8_t payload_length);
bool NRF_SET_PipeTX(NRF_Pipe pipe, NRF_AutoAcknowledge auto_ack);

/*
 * getters
 */
bool NRF_GET_PipeNumber(uint8_t* pipe);
bool NRF_GET_PayloadLength(uint8_t pipe, uint8_t* length);
bool NRF_GET_RXFifoState();
bool NRF_GET_TXFifoState();

/*
 * interrupts related
 */
void NRF_IRQ_Event(uint8_t* nrfInterruptFlag);
bool NRF_IRQ_Callback(uint8_t* nrfInterruptFlag, uint8_t* data_buffer);
bool NRF_IRQ_RxHandler(uint8_t* data_buffer);
bool NRF_IRQ_TxHandler(void);
bool NRF_IRQ_MaxHandler(void);

void NRF_PrintConfig();
#endif /* INC_NRF_NRF_H_ */
