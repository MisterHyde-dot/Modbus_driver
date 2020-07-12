#ifndef __modbus
#define __modbus
#include  "usart.h"
#include 	"crc.h"

//-----------------------------------------
#define RXTX_BUFF_SIZE					256 //recieve/transmit buffer
#define WRITE_BUFF_SIZE					240 // buffer for writing to device regs

#define READ_HOLDING_REGS				3
#define	PRESET_SINGLE_REG				6
#define PRESET_MULTPL_REGS			16
#define CMD_NOT_SET							0

#define MAX_ID_NUMBER						246
#define DEVICE_ID								1
//-----------------------------------------
#define DEST	(uint16_t*)(&ram_params) //address of regs
//-----------------------------------------

//========================================
#define UART_SPEED							115200
#define IT_FREQ									20000 // Modbus_update interrupt frequency
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
	uint16_t							timeout_tim;
	uint16_t							timeout;
	TUSART_params					usart_params;
	TModbus_TxRx_params		txrx_params;
	TModbus_Erros					error;
	TModbus_Status				status;
}TModbus;

typedef struct	Ram_params
{
	char			date_time[14]; //addr 0, 7regs
	uint32_t	usart_speed; // addr 14, 2regs
	uint32_t	usart_prev_speed;
	uint16_t	par1;
	uint16_t	par2;
	uint16_t	par3;
	uint16_t	par4;
}TRam_params;

TModbus_Status	Modbus_Slave_Rx(TModbus*);
TModbus_Status	Modbus_Slave_Tx(TModbus*);
void						Modbus_Process(TModbus*);
void 						Modbus_TxCallback(TModbus*);
void						Modbus_Data_Wait(TModbus*);
void						Modbus_Init(TModbus*,	UART_HandleTypeDef* uart_p);
void						Modbus_Update(TModbus*);

void 						Params_Init(TRam_params*);

TRam_params ram_params;
#endif
/*

*/
