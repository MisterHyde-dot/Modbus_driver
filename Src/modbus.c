/*
	modbus driver with uart8 and usart1 for stm32F427ZIt
	the principal of the driver is based on reading idle flag and if flag is true dma stream is stopped 
	and recieved data is being processed
*/

#include "modbus.h"

#define MODBUS_MAX_INSTANCES	4u

static TModbus* modbus_instances[MODBUS_MAX_INSTANCES];
static uint8_t modbus_instance_count;

static void Modbus_RegisterInstance(TModbus* p)
{
	if (p == NULL)
	{
		return;
	}

	for (uint8_t i = 0; i < modbus_instance_count; i++)
	{
		if (modbus_instances[i] == p)
		{
			return;
		}
	}

	if (modbus_instance_count < MODBUS_MAX_INSTANCES)
	{
		modbus_instances[modbus_instance_count++] = p;
	}
}

static uint32_t Modbus_ComputeTimeout(const TModbus_Config* config)
{
	if (config == NULL || config->baud_rate == 0u)
	{
		return 0u;
	}

	return (config->update_frequency * config->interframe_bits) / config->baud_rate;
}

void Modbus_Config_Init(TModbus_Config* config)
{
	if (config == NULL)
	{
		return;
	}

	config->baud_rate = MODBUS_DEFAULT_BAUDRATE;
	config->update_frequency = MODBUS_DEFAULT_UPDATE_FREQ;
	config->interframe_bits = MODBUS_INTERFRAME_BITS;
	config->max_id = MODBUS_DEFAULT_MAX_ID;
}

void Modbus_RegisterOps_Init(TModbus_RegisterOps* ops, void* context, Modbus_ReadRegister read, Modbus_WriteRegister write, uint16_t register_count)
{
	if (ops == NULL)
	{
		return;
	}

	ops->context = context;
	ops->read = read;
	ops->write = write;
	ops->register_count = register_count;
}

static bool Modbus_IsRegisterRangeValid(const TModbus* p, uint16_t address, uint16_t count)
{
	if (p == NULL)
	{
		return false;
	}

	if (p->register_ops.register_count == 0u)
	{
		return true;
	}

	if (address >= p->register_ops.register_count)
	{
		return false;
	}

	if (count > (p->register_ops.register_count - address))
	{
		return false;
	}

	return true;
}

bool Modbus_ReadRegisterValue(TModbus* p, uint16_t address, uint16_t* value)
{
	if (p == NULL || p->register_ops.read == NULL || value == NULL)
	{
		return false;
	}

	if (!Modbus_IsRegisterRangeValid(p, address, 1u))
	{
		return false;
	}

	return p->register_ops.read(p->register_ops.context, address, value);
}

bool Modbus_WriteRegisterValue(TModbus* p, uint16_t address, uint16_t value)
{
	if (p == NULL || p->register_ops.write == NULL)
	{
		return false;
	}

	if (!Modbus_IsRegisterRangeValid(p, address, 1u))
	{
		return false;
	}

	return p->register_ops.write(p->register_ops.context, address, value);
}

void Modbus_SetBaudRate(TModbus* p, uint32_t baud_rate)
{
	if (p == NULL || baud_rate == 0u)
	{
		return;
	}

	p->config.baud_rate = baud_rate;
	p->usart_params.huart_p->Init.BaudRate = baud_rate;
	p->timeout = Modbus_ComputeTimeout(&p->config);
}

TModbus_Status	Modbus_Slave_Rx(TModbus* p)
{
	uint16_t	crc;	
	uint8_t		byte_cntr = 0;
	uint8_t		max_id = (p->config.max_id == 0u) ? MODBUS_DEFAULT_MAX_ID : p->config.max_id;

	p->error.all = 0;
	p->status = data_await;
	memset(p->txrx_params.txrx_buffer, 0, sizeof(p->txrx_params.txrx_buffer));
	p->txrx_params.id = p->usart_params.rx_data[byte_cntr++];
	if (p->txrx_params.id > max_id)
	{
		p->error.bit.id_error = 1;
		p->status = id_error;
	}			
	else
	{
		p->error.bit.id_error = false;
	}
	// Command	
	p->txrx_params.cmd = p->usart_params.rx_data[byte_cntr++];
	if ( p->txrx_params.cmd != MODBUS_FUNC_PRESET_MULTIPLE_REGS && p->txrx_params.cmd != MODBUS_FUNC_PRESET_SINGLE_REG && p->txrx_params.cmd != MODBUS_FUNC_READ_HOLDING_REGS)
	{
		p->error.bit.cmd_error = true;
		p->status = cmd_error;
	}
	else
	{
		p->error.bit.cmd_error = false;
	}
	// Data address	
	p->txrx_params.data_addr = p->usart_params.rx_data[byte_cntr++]<<8;
	p->txrx_params.data_addr += p->usart_params.rx_data[byte_cntr++];

	// number of regs
	p->txrx_params.regs_number = p->usart_params.rx_data[byte_cntr++]<< 8;
	p->txrx_params.regs_number += p->usart_params.rx_data[byte_cntr++];

	if (p->txrx_params.regs_number == 0u)
	{
		p->status = data_error;
		p->error.bit.data_error = true;
	}

	if ( !p->error.all)
	{
		if (p->txrx_params.cmd == MODBUS_FUNC_PRESET_MULTIPLE_REGS )
		{
			uint32_t expected_bytes = (uint32_t)p->txrx_params.regs_number * MODBUS_REG_BYTES;
			p->txrx_params.bytes_numb =p->usart_params.rx_data[byte_cntr++];
			if (expected_bytes > WRITE_BUFF_SIZE)
			{
				p->status = data_error;
				p->error.bit.data_error = true;
			}
			else if (p->txrx_params.bytes_numb != expected_bytes)
			{
				p->status = data_error;
				p->error.bit.data_error = true;
			}
			else
			{
				for(int i = 0; i < p->txrx_params.bytes_numb; i++)
				{
					p->txrx_params.txrx_buffer[i] = p->usart_params.rx_data[byte_cntr++];
				}
			}				
		}
		else if (p->txrx_params.cmd == MODBUS_FUNC_PRESET_SINGLE_REG)
		{
			p->txrx_params.txrx_buffer[0] = p->usart_params.rx_data[byte_cntr++];
			p->txrx_params.txrx_buffer[1] = p->usart_params.rx_data[byte_cntr++];
		}
// crc
		p->txrx_params.crc_result =  p->usart_params.rx_data[byte_cntr++] << MODBUS_HIGH_BYTE_SHIFT;
		p->txrx_params.crc_result += p->usart_params.rx_data[byte_cntr++];
		crc = CRC16((uint8_t*)p->usart_params.rx_data, byte_cntr - MODBUS_CRC_BYTES);		
		if (crc != p->txrx_params.crc_result)
		{
			p->status = crc_error;
			p->error.bit.crc_error = true;
		}
		else
		{
			p->status = data_recieved;
		}	
	}
	memset((uint8_t*)p->usart_params.tx_data,0,sizeof(p->usart_params.rx_data));
	return p->status;
}

TModbus_Status	Modbus_Slave_Tx(TModbus* p)
{
	uint16_t	crc;	
	uint8_t		byte_cntr = 0;
//id
	p->usart_params.tx_data[byte_cntr++] = p->txrx_params.id;
//cmd
	p->usart_params.tx_data[byte_cntr++] = p->txrx_params.cmd;

	if ( p->txrx_params.cmd == MODBUS_FUNC_READ_HOLDING_REGS )
	{
// bytes amount
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.regs_number * MODBUS_REG_BYTES;
//	data
		for(uint16_t byte_index = 0; byte_index < p->txrx_params.regs_number * MODBUS_REG_BYTES; byte_index++)
		{
			p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[byte_index];			
		}
	}
	else if ( p->txrx_params.cmd == MODBUS_FUNC_PRESET_MULTIPLE_REGS )
	{
// Data address	
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr & 0xFF00) >> MODBUS_HIGH_BYTE_SHIFT;
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr & MODBUS_BYTE_MASK);

// number of regs
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.regs_number & 0xFF00) >> MODBUS_HIGH_BYTE_SHIFT;
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.regs_number & MODBUS_BYTE_MASK;	
	}
	else if (p->txrx_params.cmd == MODBUS_FUNC_PRESET_SINGLE_REG)
	{
//address 
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr & 0xFF00) >> MODBUS_HIGH_BYTE_SHIFT;
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.data_addr & MODBUS_BYTE_MASK;
//written data
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[0];
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[1];
	}
// crc
	crc = CRC16((uint8_t*)p->usart_params.tx_data, byte_cntr);	
	p->usart_params.tx_data[byte_cntr++] = (crc & 0xFF00) >> MODBUS_HIGH_BYTE_SHIFT;
	p->usart_params.tx_data[byte_cntr++] = crc & MODBUS_BYTE_MASK;
	p->transmit_flag = 1;
	HAL_UART_Transmit_DMA(p->usart_params.huart_p, (uint8_t*)p->usart_params.tx_data, byte_cntr);
	return p->status;
}

void	Modbus_Process(TModbus* p)
{
	if(Modbus_Slave_Rx(p) == data_recieved)
	{
		if(p->txrx_params.cmd == MODBUS_FUNC_PRESET_MULTIPLE_REGS)
		{
			if (!Modbus_IsRegisterRangeValid(p, p->txrx_params.data_addr, p->txrx_params.regs_number))
			{
				p->status = data_error;
				p->error.bit.data_error = true;
			}
			else
			{
				for(uint16_t reg_index = 0; reg_index < p->txrx_params.regs_number; reg_index++)
				{
					uint16_t buffer_index = reg_index * MODBUS_REG_BYTES;
					p->write_reg = (uint16_t)(p->txrx_params.txrx_buffer[buffer_index] << MODBUS_HIGH_BYTE_SHIFT);
					p->write_reg += p->txrx_params.txrx_buffer[buffer_index + 1u];
					if (!Modbus_WriteRegisterValue(p, (uint16_t)(p->txrx_params.data_addr + reg_index), p->write_reg))
					{
						p->status = data_error;
						p->error.bit.data_error = true;
						break;
					}
				}
			}
		}
		else if(p->txrx_params.cmd == MODBUS_FUNC_PRESET_SINGLE_REG)
		{
			p->write_reg = (uint16_t)(p->txrx_params.txrx_buffer[0] << MODBUS_HIGH_BYTE_SHIFT);
			p->write_reg += p->txrx_params.txrx_buffer[1];
			if (!Modbus_WriteRegisterValue(p, p->txrx_params.data_addr, p->write_reg))
			{
				p->status = data_error;
				p->error.bit.data_error = true;
			}
		}
		else if(p->txrx_params.cmd == MODBUS_FUNC_READ_HOLDING_REGS)
		{
			if (!Modbus_IsRegisterRangeValid(p, p->txrx_params.data_addr, p->txrx_params.regs_number))
			{
				p->status = data_error;
				p->error.bit.data_error = true;
			}
			else
			{
				for(uint16_t reg_index = 0; reg_index < p->txrx_params.regs_number; reg_index++)
				{
					uint16_t buffer_index = reg_index * MODBUS_REG_BYTES;
					if (!Modbus_ReadRegisterValue(p, (uint16_t)(p->txrx_params.data_addr + reg_index), &p->read_reg))
					{
						p->status = data_error;
						p->error.bit.data_error = true;
						break;
					}
					p->txrx_params.txrx_buffer[buffer_index] = (uint8_t)((p->read_reg & 0xFF00) >> MODBUS_HIGH_BYTE_SHIFT);
					p->txrx_params.txrx_buffer[buffer_index + 1u] = (uint8_t)(p->read_reg & MODBUS_BYTE_MASK);
				}
			}
		}
		if (p->status == data_recieved)
		{
			Modbus_Slave_Tx(p);
		}
	}		
}

void Modbus_TxCallback(TModbus* p)
{
	p->transmit_flag = 0;
	memset((uint8_t*)p->usart_params.tx_data, 0, sizeof(p->usart_params.tx_data));
	HAL_UART_Receive_DMA(p->usart_params.huart_p, (uint8_t*)p->usart_params.rx_data, RXTX_BUFF_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // standart HAL tx callback function 
{
	for (uint8_t i = 0; i < modbus_instance_count; i++)
	{
		TModbus* instance = modbus_instances[i];
		if (instance != NULL && instance->usart_params.huart_p == huart)
		{
			Modbus_TxCallback(instance);
			break;
		}
	}
}

void	Modbus_Init(TModbus* p,	UART_HandleTypeDef* uart_p, const TModbus_Config* config, const TModbus_RegisterOps* register_ops) // initing modbus variable with specified uart pointer (something like huart1 or huart8 etc.)
{
	if (p == NULL || uart_p == NULL)
	{
		return;
	}

	memset(p, 0, sizeof(TModbus));
	p->usart_params.huart_p = uart_p;
	if (config != NULL)
	{
		p->config = *config;
	}
	else
	{
		Modbus_Config_Init(&p->config);
	}

	if (register_ops != NULL)
	{
		p->register_ops = *register_ops;
	}
	else
	{
		Modbus_RegisterOps_Init(&p->register_ops, NULL, NULL, NULL, 0u);
	}

	p->usart_params.huart_p->Init.BaudRate = p->config.baud_rate;
	p->timeout = Modbus_ComputeTimeout(&p->config);
	Modbus_RegisterInstance(p);
	HAL_UART_Receive_DMA(p->usart_params.huart_p, (uint8_t*)p->usart_params.rx_data, RXTX_BUFF_SIZE);
}

void	Modbus_Data_Wait(TModbus* p)
{
	if(p->transmit_flag) // if data transmition in progress then return
		return;
	
	if((RESET != __HAL_UART_GET_FLAG(p->usart_params.huart_p, UART_FLAG_IDLE)) && p->timeout_tim++ > p->timeout) // if there is no data in usart chanel then idle flag is set and pause 3.5 symbols is counting 
	{
		 __HAL_UART_CLEAR_IDLEFLAG(p->usart_params.huart_p);                     // clearing idle flag
		HAL_UART_DMAStop(p->usart_params.huart_p); 															// stopping dma stream
		uint8_t data_length  = RXTX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(p->usart_params.huart_p->hdmarx); // the length of recived data 
		Modbus_Process(p);	// processing the recieved data
	}
	else
		p->timeout = 0;
}

void	Modbus_Update(TModbus* p) //call this function with some frequency not lower then 20kHz
{
	Modbus_Data_Wait(p);		
}
//------------------------------------------------------------
