/*
 * nrf.h
 *
 *  Created on: 29 wrz 2022
 *      Author: jasie
 */

#ifndef INC_NRF_NRF_DEFINES_H_
#define INC_NRF_NRF_DEFINES_H_

extern SPI_HandleTypeDef hspi2;		// for passing it to the NRF typedef

/*
 * COMMANDS
 */

#define	NRF_CMD_R_REGISTER  	(uint8_t)0x00	// read command and status registers (5 last bits address)
#define	NRF_CMD_W_REGISTER  	(uint8_t)0x02	// write command and status registers (5 last bits address)
#define	NRF_CMD_R_RX_PAYLOAD    (uint8_t)0x61	// read rx payload
#define	NRF_CMD_W_TX_PAYLOAD    (uint8_t)0xA0	// write tx payload
#define NRF_CMD_FLUSH_TX		(uint8_t)0xE1	// flush tx FIFO
#define NRF_CMD_FLUSH_RX		(uint8_t)0xE2	// flush rx FIFO
#define NRF_CMD_REUSE_TX_PL		(uint8_t)0xE3	// reuse last transmitted payload
#define NRF_CMD_ACTIVATE		(uint8_t)0x50	// used with data 0x73; activates regs for commands below:
#define NRF_CMD_R_RX_PL_WID		(uint8_t)0x60	// read rx payload width
#define NRF_CMD_W_ACK_PAYLOAD	(uint8_t)0xA8	// write payload to be with ack packet on pipe: (3 last bits)
#define NRF_CMD_W_TX_PAYLOAD_NOACK  (uint8_t)0xB0	//disable auto ack in tx
#define NRF_CMD_NOP				(uint8_t)0xFF	// no operation

/*
 * REGISTERS
 */

#define NRF_REG_CONFIG			(uint8_t)0x00	// configuration register
#define NRF_REG_EN_AA			(uint8_t)0x01	// enable auto acknowledgment
#define NRF_REG_EN_RXADDR		(uint8_t)0x02	// enable RX addresses
#define NRF_REG_SETUP_AW		(uint8_t)0x03	// setup of address widths
#define NRF_REG_SETUP_RETR		(uint8_t)0x04	// setup of auto retransmission
#define NRF_REG_RF_CH			(uint8_t)0x05	// rf channel
#define NRF_REG_RF_SETUP		(uint8_t)0x06	// rf setup register
#define NRF_REG_STATUS			(uint8_t)0x07	// status register
#define NRF_REG_OBSERVE_TX		(uint8_t)0x08	// tx observe register (lost packets)
#define NRF_REG_CD				(uint8_t)0x09	// carrier detect
#define NRF_REG_RX_ADDR_P0		(uint8_t)0x0A	// receive address data pipe 0
#define NRF_REG_RX_ADDR_P1		(uint8_t)0x0B	// receive address data pipe 1
#define NRF_REG_RX_ADDR_P2		(uint8_t)0x0C	// receive address data pipe 2
#define NRF_REG_RX_ADDR_P3		(uint8_t)0x0D	// receive address data pipe 3
#define NRF_REG_RX_ADDR_P4		(uint8_t)0x0E	// receive address data pipe 4
#define NRF_REG_RX_ADDR_P5		(uint8_t)0x0F	// receive address data pipe 5
#define NRF_REG_TX_ADDR			(uint8_t)0x10	// transmit address (ptx only)
#define NRF_REG_RX_PW_P0		(uint8_t)0x11	// number of bytes in rx payload data pipe 0
#define NRF_REG_RX_PW_P1		(uint8_t)0x12	// number of bytes in rx payload data pipe 1
#define NRF_REG_RX_PW_P2		(uint8_t)0x13	// number of bytes in rx payload data pipe 2
#define NRF_REG_RX_PW_P3		(uint8_t)0x14	// number of bytes in rx payload data pipe 3
#define NRF_REG_RX_PW_P4		(uint8_t)0x15	// number of bytes in rx payload data pipe 4
#define NRF_REG_RX_PW_P5		(uint8_t)0x16	// number of bytes in rx payload data pipe 5
#define NRF_REG_FIFO_STATUS		(uint8_t)0x17	// fifo status register
#define NRF_REG_DYNPD			(uint8_t)0x1C	// enable dynamic payload length
#define NRF_REG_FEATURE			(uint8_t)0x1D	// feature register

/*
 * MASKS
 */

#define NRF_MASK_RW_REG			(uint8_t)0x1F	// mask for address bits in R/W reg commands
#define NRF_MASK_W_ACK			(uint8_t)0x07	// mask for pipe number for W_ACK_PAYLOAD command
#define NRF_MASK_IRQ			(uint8_t)0x70	// mask for IRQ bits in config reg
#define NRF_MASK_CRC 			(uint8_t)0x0E	// mask for CRC settings in config reg
#define NRF_MASK_MODE			(uint8_t)0x01	// mask for switching between RX/TX
#define NRF_MASK_EN_P0			(uint8_t)0x01	// mask for enabling pipe 0 for couple of commands/registers
#define NRF_MASK_EN_P1			(uint8_t)0x02	// mask for enabling pipe 1 for couple of commands/registers
#define NRF_MASK_EN_P2			(uint8_t)0x04	// mask for enabling pipe 2 for couple of commands/registers
#define NRF_MASK_EN_P3			(uint8_t)0x08	// mask for enabling pipe 3 for couple of commands/registers
#define NRF_MASK_EN_P4			(uint8_t)0x10	// mask for enabling pipe 4 for couple of commands/registers
#define NRF_MASK_EN_P5			(uint8_t)0x20	// mask for enabling pipe 5 for couple of commands/registers
#define	NRF_MASK_AW				(uint8_t)0x03	// mask for address width field
#define NRF_MASK_ARD			(uint8_t)0xF0	// mask for auto retransmission delay
#define NRF_MASK_ARC			(uint8_t)0x0F	// mask for auto retransmission count
#define NRF_MASK_RF_CH			(uint8_t)0x7F	// mask for setting frequency
#define NRF_MASK_PLL_LOCK		(uint8_t)0x10	// mask for pll lock enable in rf setup reg
#define NRF_MASK_RF_DR			(uint8_t)0x08	// mask for setting air data rate in rf setup reg
#define NRF_MASK_RF_PWR			(uint8_t)0x06	// mask for setting power in tx mode, in rf setup reg
#define NRF_MASK_LNA			(uint8_t)0x01	// mask for setup of LNA gain
#define NRF_MASK_RX_DR			(uint8_t)0x40	// mask for data ready rx fifo bit
#define NRF_MASK_TX_DS			(uint8_t)0x20	// mask for data sent tx fifo interrupt
#define NRF_MASK_MAX_RT			(uint8_t)0x10	// mask for max number of tx retransmissions.
												// If set must be cleared before further communication
#define NRF_MASK_RX_P_NO		(uint8_t)0x0E	// mask for data pipe for the payload available to read from rx fifo
#define NRF_MASK_TX_FULL		(uint8_t)0x01	// mask for tx fifo full flag
#define NRF_MASK_PLOS_CNT		(uint8_t)0xF0	// mask for counter of lost packets
#define NRF_MASK_ARC_CNT		(uint8_t)0x0F	// mask for counter of retransmitted packets
#define NRF_MASK_CD				(uint8_t)0x01	// mask for carrier detect
#define NRF_MASK_RX_PW_P		(uint8_t)0x10	// mask for number of bytes in rx payload for given pipe
#define NRF_MASK_TX_REUSE		(uint8_t)0x40	// mask for flag to reuse last transmitted packet
#define NRF_MASK_TX_FULL		(uint8_t)0x20	// mask for tx fifo full flag
#define NRF_MASK_TX_EMPTY		(uint8_t)0x10	// mask for tx fifo empty flag
#define NRF_MASK_RX_FULL		(uint8_t)0x02	// mask for rx fifo full flag
#define NRF_MASK_RX_EMPTY		(uint8_t)0x01	// mask for rx fifo empty flag
#define NRF_MASK_EN_DPL			(uint8_t)0x04	// mask for dynamic payload length enable
#define NRF_MASK_EN_ACK_PAY		(uint8_t)0x02	// mask for payload with ack enable
#define NRF_MASK_EN_DYN_ACK		(uint8_t)0x01	// mask for W_TX_PAYLOAD_NOACK command enable

/*
 * BIT STATES
 */

#define NRF_BIT_SET_HIGH 1
#define NRF_BIT_SET_LOW 0

/*
 * ENUMERATIONS
 */

typedef enum{
	ILLEGAL = 0x00,
	THREE = 0x01,
	FOUR = 0x02,
	FIVE = 0x03
}NRF_AddressWidth;

/*
 *
 */

typedef enum{
	DELAY_250uS = 0x00,
	DELAY_500uS = 0x01,
	DELAY_750uS = 0x02,
	DELAY_1000uS = 0x03,
	DELAY_1250uS = 0x04,
	DELAY_1500uS = 0x05,
	DELAY_1750uS = 0x06,
	DELAY_2000uS = 0x07,
	DELAY_2250uS = 0x08,
	DELAY_2500uS = 0x09,
	DELAY_2750uS = 0x0A,
	DELAY_3000uS = 0x0B,
	DELAY_3250uS = 0x0C,
	DELAY_3500uS = 0x0D,
	DELAY_3750uS = 0x0E,
	DELAY_4000uS = 0x0F
}NRF_RetransmitDelay;

typedef enum{
	RATE_1Mbps = 0x00,
	RATE_2Mbps = 0x01
}NRF_AirDataRate;

typedef uint8_t NRF_Frequency;

typedef enum{
	dBm_0 = 0x03,
	dBm_6 = 0x02,
	dBm_12 = 0x01,
	dBm_18 = 0x00
}NRF_PowerAmplifier;

typedef enum{
	LOW = 0x00,
	HIGH = 0x01
}NRF_LNAsetup;

typedef enum{
	RX = 0x01,
	TX = 0x00
}NRF_Mode;

typedef enum{
	OFF = 0x00,
	ON = 0x01
}NRF_DynamicPayload;

typedef enum{
	ONE = 0x00,
	TWO = 0x01
}NRF_CRCbytes;

typedef enum{
	ENABLE = 0x01,
	DISABLE = 0x00
}NRF_CRC;

typedef enum{
	OFF = 0x00,
	ON = 0x01
}NRF_AutoAcknowledge;

typedef enum{
	OFF = 0x00,
	ONE = 0x01,
	TWO = 0x02,
	THREE = 0x03,
	FOUR = 0x04,
	FIVE = 0x05,
	SIX = 0x06,
	SEVEN = 0x07,
	EIGHT = 0x08,
	NINE = 0x09,
	TEN = 0x0A,
	ELEVEN = 0x0B,
	TWELVE = 0x0C,
	THIRTEEN = 0x0D,
	FOURTEEN = 0x0E,
	FIFTEEN = 0x0F
}NRF_RetransmitCount;

typedef struct{
	SPI_HandleTypedef* spi_handle_;
	NRF_AirDataRate rate_;
	NRF_Frequency frequency_;
	NRF_PowerAmplifier power_amp_;
	NRF_LNAsetup lna_;
	NRF_Mode mode_;
	NRF_DynamicPayload dpl_;
	NRF_CRC crc_;
	NRF_CRCbytes crc_bytes_;
	NRF_AddressWidth address_width_;
	NRF_AutoAcknowledge ack_;
	NRF_RetransmitCount retransmissions_;
	NRF_RetransmitDelay ret_delay_;
}NRF_HandleTypedef;

#endif /* INC_NRF_NRF_DEFINES_H_ */
