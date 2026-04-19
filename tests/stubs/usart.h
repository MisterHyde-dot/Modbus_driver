#ifndef MODBUS_TEST_USART_H
#define MODBUS_TEST_USART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef __IO
#define __IO volatile
#endif

#ifndef RESET
#define RESET 0u
#endif

#ifndef SET
#define SET 1u
#endif

#ifndef UART_FLAG_IDLE
#define UART_FLAG_IDLE 0x00000010u
#endif

typedef struct UART_InitTypeDef
{
	uint32_t BaudRate;
} UART_InitTypeDef;

typedef struct DMA_HandleTypeDef
{
	uint16_t counter;
} DMA_HandleTypeDef;

typedef struct UART_HandleTypeDef
{
	UART_InitTypeDef Init;
	DMA_HandleTypeDef* hdmarx;
	void* Instance;
} UART_HandleTypeDef;

#define __HAL_UART_GET_FLAG(huart, flag) (RESET)
#define __HAL_UART_CLEAR_IDLEFLAG(huart) ((void)(huart))
#define __HAL_DMA_GET_COUNTER(handle) ((handle) ? (handle)->counter : 0u)

void HAL_UART_DMAStop(UART_HandleTypeDef* huart);
void HAL_UART_Receive_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size);
void HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size);

#endif
