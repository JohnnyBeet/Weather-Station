/*
 * uart_dma.h
 *
 *  Created on: Nov 23, 2022
 *  Edited by: JohnnyBeet
 */

/*
Copyright (c) 2018 Mateusz Salamon <mateusz@msalamon.pl>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOF

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
