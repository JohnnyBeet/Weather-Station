/*
 * uart_dma.h
 *
 *  Created on: Nov 23, 2022
 *      Author: jasie
 */

#ifndef INC_UART_DMA_H_
#define INC_UART_DMA_H_

#include "stm32f4xx_hal.h"

#define DMA_RX_BUFFER_SIZE          128
#define UART_BUFFER_SIZE            1024

typedef struct
{
	UART_HandleTypeDef* huart;					// UART handler

	uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];	// DMA direct buffer
	uint8_t UART_Buffer[UART_BUFFER_SIZE];		// UART working circular buffer

	uint16_t UartBufferHead;
	uint16_t UartBufferTail;
}UARTDMA_HandleTypeDef;

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DmaIrqHandler(UARTDMA_HandleTypeDef *huartdma);

uint8_t UARTDMA_IsDataReady(UARTDMA_HandleTypeDef *huartdma);
int UARTDMA_GetCharFromBuffer(UARTDMA_HandleTypeDef *huartdma);
//int UARTDMA_GetLineFromBuffer(UARTDMA_HandleTypeDef *huartdma, char *OutBuffer);

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart);
#endif /* INC_UART_DMA_H_ */
