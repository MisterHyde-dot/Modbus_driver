#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "modbus.h"

static uint8_t last_tx_buffer[RXTX_BUFF_SIZE];
static uint16_t last_tx_length;
static UART_HandleTypeDef* last_tx_uart;

typedef struct
{
	uint16_t* regs;
	uint16_t count;
} TestRegisterContext;

static void WriteUint16BE(uint8_t* buffer, uint16_t* index, uint16_t value);
static void AssertCrcAt(const uint8_t* buffer, uint16_t data_length, uint16_t crc_index);

void HAL_UART_DMAStop(UART_HandleTypeDef* huart)
{
	(void)huart;
}

void HAL_UART_Receive_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size)
{
	(void)huart;
	(void)data;
	(void)size;
}

void HAL_UART_Transmit_DMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size)
{
	last_tx_uart = huart;
	last_tx_length = size;
	memset(last_tx_buffer, 0, sizeof(last_tx_buffer));
	if (size > 0u)
	{
		memcpy(last_tx_buffer, data, size);
	}
}

uint16_t CRC16(const uint8_t* data, uint16_t length)
{
	uint16_t crc = 0xFFFFu;
	for (uint16_t i = 0; i < length; i++)
	{
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8u; bit++)
		{
			if (crc & 1u)
			{
				crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
			}
			else
			{
				crc >>= 1u;
			}
		}
	}
	return crc;
}

static bool TestReadRegister(void* context, uint16_t address, uint16_t* value)
{
	TestRegisterContext* ctx = (TestRegisterContext*)context;
	if (ctx == NULL || value == NULL || address >= ctx->count)
	{
		return false;
	}
	*value = ctx->regs[address];
	return true;
}

static bool TestWriteRegister(void* context, uint16_t address, uint16_t value)
{
	TestRegisterContext* ctx = (TestRegisterContext*)context;
	if (ctx == NULL || address >= ctx->count)
	{
		return false;
	}
	ctx->regs[address] = value;
	return true;
}

static void BuildRequest(uint8_t* buffer,
	uint8_t id,
	uint8_t cmd,
	uint16_t address,
	uint16_t regs,
	const uint8_t* payload,
	uint8_t payload_len,
	bool include_byte_count,
	uint16_t* out_length)
{
	uint16_t index = 0;
	buffer[index++] = id;
	buffer[index++] = cmd;
	WriteUint16BE(buffer, &index, address);
	WriteUint16BE(buffer, &index, regs);
	if (payload_len > 0u)
	{
		if (include_byte_count)
		{
			buffer[index++] = payload_len;
		}
		memcpy(&buffer[index], payload, payload_len);
		index += payload_len;
	}
	uint16_t crc = CRC16(buffer, index);
	WriteUint16BE(buffer, &index, crc);
	*out_length = index;
}

static void BuildSingleRegisterRequest(uint8_t* buffer,
	uint8_t id,
	uint16_t address,
	uint16_t value,
	uint16_t* out_length)
{
	uint8_t payload[2];

	uint16_t payload_index = 0u;

	WriteUint16BE(payload, &payload_index, value);
	BuildRequest(buffer,
		id,
		MODBUS_FUNC_PRESET_SINGLE_REG,
		address,
		1u,
		payload,
		(uint8_t)sizeof(payload),
		false,
		out_length);
}

static void WriteUint16BE(uint8_t* buffer, uint16_t* index, uint16_t value)
{
	buffer[(*index)++] = (uint8_t)(value >> MODBUS_HIGH_BYTE_SHIFT);
	buffer[(*index)++] = (uint8_t)(value & MODBUS_BYTE_MASK);
}

static void AssertCrcAt(const uint8_t* buffer, uint16_t data_length, uint16_t crc_index)
{
	uint16_t crc = CRC16(buffer, data_length);
	assert(buffer[crc_index] == (uint8_t)(crc >> MODBUS_HIGH_BYTE_SHIFT));
	assert(buffer[crc_index + 1u] == (uint8_t)(crc & MODBUS_BYTE_MASK));
}

static void ResetTransmitCapture(void)
{
	memset(last_tx_buffer, 0, sizeof(last_tx_buffer));
	last_tx_length = 0u;
	last_tx_uart = NULL;
}

static void SetupModbus(TModbus* modbus, UART_HandleTypeDef* uart, TestRegisterContext* ctx)
{
	TModbus_Config config;
	TModbus_RegisterOps ops;

	Modbus_Config_Init(&config);
	Modbus_RegisterOps_Init(&ops, ctx, TestReadRegister, TestWriteRegister, ctx->count);
	Modbus_Init(modbus, uart, &config, &ops);
}

static void TestReadHoldingRegisters(void)
{
	uint16_t regs[] = {0x1234u, 0xABCDu, 0x0011u};
	TestRegisterContext ctx = {.regs = regs, .count = 3u};
	TModbus modbus;
	UART_HandleTypeDef uart = {0};

	uint8_t request[16] = {0};
	uint16_t request_length = 0u;

	SetupModbus(&modbus, &uart, &ctx);
	BuildRequest(request, 1u, MODBUS_FUNC_READ_HOLDING_REGS, 1u, 2u, NULL, 0u, false, &request_length);
	memcpy((uint8_t*)modbus.usart_params.rx_data, request, request_length);

	ResetTransmitCapture();
	Modbus_Process(&modbus);

	assert(last_tx_uart == &uart);
	assert(last_tx_length == 9u);
	assert(last_tx_buffer[0] == 1u);
	assert(last_tx_buffer[1] == MODBUS_FUNC_READ_HOLDING_REGS);
	assert(last_tx_buffer[2] == 4u);
	assert(last_tx_buffer[3] == 0xABu);
	assert(last_tx_buffer[4] == 0xCDu);
	assert(last_tx_buffer[5] == 0x00u);
	assert(last_tx_buffer[6] == 0x11u);

	AssertCrcAt(last_tx_buffer, 7u, 7u);
}

static void TestWriteSingleRegister(void)
{
	uint16_t regs[] = {0x0000u, 0x0000u, 0x0000u};
	TestRegisterContext ctx = {.regs = regs, .count = 3u};
	TModbus modbus;
	UART_HandleTypeDef uart = {0};
	uint8_t request[16] = {0};
	uint16_t request_length = 0u;

	SetupModbus(&modbus, &uart, &ctx);
	BuildSingleRegisterRequest(request, 1u, 2u, 0xBEEFu, &request_length);
	memcpy((uint8_t*)modbus.usart_params.rx_data, request, request_length);

	ResetTransmitCapture();
	Modbus_Process(&modbus);

	assert(regs[2] == 0xBEEFu);
	assert(last_tx_length == 8u);
	assert(last_tx_buffer[0] == 1u);
	assert(last_tx_buffer[1] == MODBUS_FUNC_PRESET_SINGLE_REG);
	assert(last_tx_buffer[2] == 0x00u);
	assert(last_tx_buffer[3] == 0x02u);
	assert(last_tx_buffer[4] == 0xBEu);
	assert(last_tx_buffer[5] == 0xEFu);

	AssertCrcAt(last_tx_buffer, 6u, 6u);
}

static void TestWriteMultipleRegisters(void)
{
	uint16_t regs[] = {0x0000u, 0x0000u, 0x0000u};
	TestRegisterContext ctx = {.regs = regs, .count = 3u};
	TModbus modbus;
	UART_HandleTypeDef uart = {0};
	uint8_t request[20] = {0};
	uint8_t payload[] = {0x00u, 0xAAu, 0x00u, 0xBBu};
	uint16_t request_length = 0u;

	SetupModbus(&modbus, &uart, &ctx);
	BuildRequest(request, 1u, MODBUS_FUNC_PRESET_MULTIPLE_REGS, 0u, 2u, payload, sizeof(payload), true, &request_length);
	memcpy((uint8_t*)modbus.usart_params.rx_data, request, request_length);

	ResetTransmitCapture();
	Modbus_Process(&modbus);

	assert(regs[0] == 0x00AAu);
	assert(regs[1] == 0x00BBu);
	assert(last_tx_length == 8u);
	assert(last_tx_buffer[0] == 1u);
	assert(last_tx_buffer[1] == MODBUS_FUNC_PRESET_MULTIPLE_REGS);
	assert(last_tx_buffer[2] == 0x00u);
	assert(last_tx_buffer[3] == 0x00u);
	assert(last_tx_buffer[4] == 0x00u);
	assert(last_tx_buffer[5] == 0x02u);

	AssertCrcAt(last_tx_buffer, 6u, 6u);
}

int main(void)
{
	TestReadHoldingRegisters();
	TestWriteSingleRegister();
	TestWriteMultipleRegisters();
	return 0;
}
