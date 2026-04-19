#ifndef __modbus
#define __modbus
#include  "usart.h"
#include 	"crc.h"

//-----------------------------------------
#define RXTX_BUFF_SIZE					256 //recieve/transmit buffer
#define WRITE_BUFF_SIZE					240 // buffer for writing to device regs

#define MODBUS_FUNC_READ_HOLDING_REGS				3u
#define	MODBUS_FUNC_PRESET_SINGLE_REG				6u
#define MODBUS_FUNC_PRESET_MULTIPLE_REGS			16u
#define MODBUS_CMD_NOT_SET										0u

#define READ_HOLDING_REGS									MODBUS_FUNC_READ_HOLDING_REGS
#define	PRESET_SINGLE_REG									MODBUS_FUNC_PRESET_SINGLE_REG
#define PRESET_MULTPL_REGS									MODBUS_FUNC_PRESET_MULTIPLE_REGS
#define CMD_NOT_SET													MODBUS_CMD_NOT_SET

#define MODBUS_DEFAULT_MAX_ID							246u
#define MODBUS_DEFAULT_DEVICE_ID					1u
//-----------------------------------------

//========================================
#define MODBUS_DEFAULT_BAUDRATE				115200u
#define MODBUS_DEFAULT_UPDATE_FREQ		20000u // Modbus_update interrupt frequency
#define MODBUS_INTERFRAME_BITS				28u
#define MODBUS_REG_BYTES							2u
#define MODBUS_CRC_BYTES							2u
#define MODBUS_HIGH_BYTE_SHIFT				8u
#define MODBUS_BYTE_MASK							0xFFu
//========================================
typedef enum	_Modbus_Status
{
	data_await				= 0,
	data_recieved			= 1,
	data_transmited		= 2,
	cmd_error					= 3,
	id_error					= 4,
	data_recvieving 	= 5,
	data_transmiting  = 6,
	crc_error					= 7,
	data_error				= 8
}TModbus_Status;

typedef struct USART_params{
	UART_HandleTypeDef* 	huart_p;
	
	__IO uint8_t 					rx_data[RXTX_BUFF_SIZE];
	__IO uint8_t 					tx_data[RXTX_BUFF_SIZE];
	
	bool 					callback;	
	
}TUSART_params;

typedef struct Modbus_TxRx_params
{
	uint8_t		id;
	uint8_t		cmd;
	uint8_t		bytes_numb;
	uint8_t		txrx_buffer[WRITE_BUFF_SIZE];
	uint16_t	data_addr;
	uint16_t	regs_number;
	uint16_t 	crc_result;
	
	
}TModbus_TxRx_params;

typedef bool (*Modbus_ReadRegister)(void* context, uint16_t address, uint16_t* value);
typedef bool (*Modbus_WriteRegister)(void* context, uint16_t address, uint16_t value);

typedef struct Modbus_RegisterOps
{
	void*									context;
	Modbus_ReadRegister		read;
	Modbus_WriteRegister	write;
	uint16_t							register_count;
}TModbus_RegisterOps;

typedef struct Modbus_Config
{
	uint32_t	baud_rate;
	uint32_t	update_frequency;
	uint16_t	interframe_bits;
	uint8_t		max_id;
}TModbus_Config;

typedef union Modbus_Erros
{
	uint8_t all;
	struct
	{
		uint8_t			id_error:		1;
		uint8_t			cmd_error:	1;
		uint8_t			data_error:	1;
		uint8_t			crc_error:	1;
		uint8_t			reserv:		  4;
	}bit;

}TModbus_Erros;

typedef struct Modbus
{
	uint8_t								transmit_flag;
	uint8_t								slave_rx_state;
	uint16_t							write_reg;
	uint16_t							read_reg;
	uint32_t							timeout_tim;
	uint32_t							timeout;
	TUSART_params					usart_params;
	TModbus_TxRx_params		txrx_params;
	TModbus_Erros					error;
	TModbus_Status				status;
	TModbus_Config				config;
	TModbus_RegisterOps		register_ops;
}TModbus;

TModbus_Status	Modbus_Slave_Rx(TModbus*);
TModbus_Status	Modbus_Slave_Tx(TModbus*);
void						Modbus_Process(TModbus*);
void 						Modbus_TxCallback(TModbus*);
void						Modbus_Data_Wait(TModbus*);
void						Modbus_Init(TModbus*,	UART_HandleTypeDef* uart_p, const TModbus_Config* config, const TModbus_RegisterOps* register_ops);
void						Modbus_Update(TModbus*);

void						Modbus_Config_Init(TModbus_Config*);
void						Modbus_RegisterOps_Init(TModbus_RegisterOps*, void* context, Modbus_ReadRegister read, Modbus_WriteRegister write, uint16_t register_count);
bool						Modbus_ReadRegisterValue(TModbus*, uint16_t address, uint16_t* value);
bool						Modbus_WriteRegisterValue(TModbus*, uint16_t address, uint16_t value);
void						Modbus_SetBaudRate(TModbus*, uint32_t baud_rate);

#endif
/*

*/
