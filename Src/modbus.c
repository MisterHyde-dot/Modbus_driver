/*
	modbus driver with uart8 and usart1 for stm32F427ZIt
	the principal of the driver is based on reading idle flag and if flag is true dma stream is stopped 
	and recieved data is being processed
*/

#include "modbus.h"
#include "project_ver.h"
#include "stdio.h"
TModbus usart1;
TModbus	uart8;

TModbus_Status	Modbus_Slave_Rx(TModbus* p)
{
	uint16_t	crc;	
	uint8_t		byte_cntr = 0;
	memset(p->txrx_params.txrx_buffer, 0, sizeof(p->txrx_params.txrx_buffer));
	p->txrx_params.id = p->usart_params.rx_data[byte_cntr++];
	if (p->txrx_params.id > MAX_ID_NUMBER)
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
	if ( p->txrx_params.cmd != PRESET_MULTPL_REGS && p->txrx_params.cmd != PRESET_SINGLE_REG && p->txrx_params.cmd != READ_HOLDING_REGS)
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

	if ( !p->error.all)
	{
		if (p->txrx_params.cmd == PRESET_MULTPL_REGS )
		{
			p->txrx_params.bytes_numb =p->usart_params.rx_data[byte_cntr++];
			if(p->txrx_params.bytes_numb > WRITE_BUFF_SIZE)
			{
				p->status = data_error;
			}
			else
			{
				for(int i = 0; i < p->txrx_params.bytes_numb; i++)
				{
					p->txrx_params.txrx_buffer[i] = p->usart_params.rx_data[byte_cntr++];
				}
			}				
		}
		else if (p->txrx_params.cmd == PRESET_SINGLE_REG)
		{
			p->txrx_params.txrx_buffer[0] = p->usart_params.rx_data[byte_cntr++];
			p->txrx_params.txrx_buffer[1] = p->usart_params.rx_data[byte_cntr++];
		}
// crc
		p->txrx_params.crc_result =  p->usart_params.rx_data[byte_cntr++] << 8;
		p->txrx_params.crc_result += p->usart_params.rx_data[byte_cntr++];
		crc = CRC16((uint8_t*)p->usart_params.rx_data, byte_cntr - 2);		
		if (crc != p->txrx_params.crc_result)
		{
			p->status = crc_error;
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

	if ( p->txrx_params.cmd == READ_HOLDING_REGS )
	{
// bytes amount
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.regs_number<<1;
//	data
		for(uint8_t i = 0; i < p->txrx_params.regs_number<<1; i++)
		{
			p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[i];			
		}
	}
	else if ( p->txrx_params.cmd == PRESET_MULTPL_REGS )
	{
// Data address	
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr&0xFF00)>>8;
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr&0xFF);

// number of regs
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.regs_number&0xFF00)>>8;
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.regs_number&0xFF;	
	}
	else if (p->txrx_params.cmd == PRESET_SINGLE_REG)
	{
//address 
		p->usart_params.tx_data[byte_cntr++] = (p->txrx_params.data_addr&0xFF00)>>8;
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.data_addr&0xFF;
//written data
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[0];
		p->usart_params.tx_data[byte_cntr++] = p->txrx_params.txrx_buffer[1];
	}
// crc
	crc = CRC16((uint8_t*)p->usart_params.rx_data, byte_cntr - 2);	
	p->usart_params.tx_data[byte_cntr++] = (crc&0xFF00)>>8;
	p->usart_params.tx_data[byte_cntr++] = crc&0xFF;
	p->transmit_flag = 1;
	HAL_UART_Transmit_DMA(p->usart_params.huart_p, (uint8_t*)p->usart_params.tx_data, byte_cntr);
	return p->status;
}

void	Modbus_Process(TModbus* p)
{
	uint16_t* data;
	uint16_t* dest;
	
	if(Modbus_Slave_Rx(p) == data_recieved)
	{
		if(p->txrx_params.cmd == PRESET_MULTPL_REGS)
		{
			int i = 0;
			while(i < p->txrx_params.bytes_numb)
			{
				dest = DEST + p->txrx_params.data_addr + i;
				p->write_reg = p->txrx_params.txrx_buffer[i++]<<8;
				p->write_reg += p->txrx_params.txrx_buffer[i++];
				data = &p->write_reg;
				memcpy(dest, data, 2);
			}
		}
		else if(p->txrx_params.cmd == PRESET_SINGLE_REG)
		{
				dest = DEST + p->txrx_params.data_addr;
				p->write_reg = p->txrx_params.txrx_buffer[0]<<8;
				p->write_reg += p->txrx_params.txrx_buffer[1];
				data = &p->write_reg;
				memcpy(dest, data, 2);
		}
		else if(p->txrx_params.cmd == READ_HOLDING_REGS)
		{
			uint16_t i = 0;
			while(i < (p->txrx_params.regs_number<<1))
			{
				dest = DEST + (p->txrx_params.data_addr + i);
				p->read_reg = *dest;
				p->txrx_params.txrx_buffer[i++] = (p->read_reg&0xFF00)>>8;
				p->txrx_params.txrx_buffer[i++] = (p->read_reg&0xFF);
			}
		}
		Modbus_Slave_Tx(p);
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
	if (huart->Instance == USART1)
		Modbus_TxCallback(&usart1);	
	else if (huart->Instance == UART8)
		Modbus_TxCallback(&uart8);
}

void	Modbus_Init(TModbus* p,	UART_HandleTypeDef* uart_p) // initing modbus variable with specified uart pointer (something like huart1 or huart8 etc.)
{
	memset(p, 0, sizeof(TModbus));
	p->usart_params.huart_p = uart_p;
	p->usart_params.huart_p->Init.BaudRate = UART_SPEED;
	p->timeout = (IT_FREQ*28)/p->usart_params.huart_p->Init.BaudRate;
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
	if(ram_params.usart_prev_speed != ram_params.usart_speed) 
	{
		switch(ram_params.usart_speed)
		{
			case 115200:
				p->usart_params.huart_p->Init.BaudRate = ram_params.usart_speed;
			break;
			
			case 57600:
				p->usart_params.huart_p->Init.BaudRate = ram_params.usart_speed;
			break;
			
			case 19200:
				p->usart_params.huart_p->Init.BaudRate = ram_params.usart_speed;
			break;
			
			case 9600:
				p->usart_params.huart_p->Init.BaudRate = ram_params.usart_speed;
			break;
		}
		p->timeout = (IT_FREQ*28)/p->usart_params.huart_p->Init.BaudRate;
	}
}

void	Params_Init(TRam_params* p)
{
	sprintf(p->date_time, "%d . %d . %d , %d : %d",VERSION_DAY, VERSION_MONTH, VERSION_YEAR, VERSION_HOUR, VERSION_MIN);
}
//------------------------------------------------------------
