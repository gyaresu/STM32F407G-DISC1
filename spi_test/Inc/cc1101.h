#ifndef __CC_1101__
#define __CC_1101__

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define RECEIVER_MODULE_ADDRESS									0xAA

#define FIFO_ACCESS															0x3F

/* command strobs definition */
#define SRES_STROBE															0x30
#define SFSTXON																	0x31
#define SXOFF																		0x32
#define SCAL																		0x33
#define SRX																			0x34
#define STX																			0x35
#define SIDLE																		0x36
#define SWOR																		0x38
#define SPWD																		0x39
#define SFRX																		0x3A
#define SFTX																		0x3B
#define SWORRST																	0x3C
#define SNOP																		0x3D

/* status registers definition*/
#define PARTNUM																	0x30
#define VERSION																	0x31
#define FREQEST																	0x32
#define LQI																			0x33
#define RSSI																		0x34
#define MARCSTATE																0x35
#define WORTIME1																0x36
#define WORTIME0																0x37
#define PKTSTATUS																0x38
#define VCO_VC_DAC															0x39
#define TXBYTES																	0x3A
#define RXBYTES																	0x3B
#define RCCTRL1_STATUS													0x3C
#define RCCTRL0_STATUS													0x3D

/* configuration registers definition */
#define IOCFG2																	0x00
#define IOCFG1																	0x01
#define IOCFG0																	0x02
#define FIFOTHR																	0x03
#define SYNC1																		0x04
#define SYNC0																		0x05
#define PKTLEN																	0x06
#define PKTCTRL1																0x07
#define PKTCTRL0																0x08
#define ADDR																		0x09
#define CHANNR																	0x0A
#define FSCTRL1																	0x0B
#define FSCTRL0																	0x0C
#define FREQ2																		0x0D
#define FREQ1																		0x0E
#define FREQ0																		0x0F
#define MDMCFG4																	0x10
#define MDMCFG3																	0x11
#define MDMCFG2																	0x12
#define MDMCFG1																	0x13
#define MDMCFG0																	0x14
#define DEVIATN																	0x15
#define MCSM2																		0x16
#define MCSM1																		0x17
#define MCSM0																		0x18
#define FOCCFG																	0x19
#define BSCFG																		0x1A
#define AGCCTRL2																0x1B
#define AGCCTRL1																0x1C
#define AGCCTRL0																0x1D
#define WOREVT1																	0x1E
#define WOREVT0																	0x1F
#define WORCTRL																	0x20
#define FREND1																	0x21
#define FREND0																	0x22
#define FSCAL3																	0x23
#define FSCAL2																	0x24
#define FSCAL1																	0x25
#define FSCAL0																	0x26
#define RCCTRL1																	0x27
#define RCCTRL0																	0x28
#define FSTEST																	0x29
#define PTEST																		0x2A
#define AGCTEST																	0x2B
#define TEST2																		0x2C
#define TEST1																		0x2D
#define TEST0																		0x2E

#define _MODULE_SELECT		HAL_GPIO_WritePin(cc1101_CSN_GPIO_Port, cc1101_CSN_Pin, GPIO_PIN_RESET);

#define WAIT_CHP_RDY			while(HAL_GPIO_ReadPin(cc1101_SO_GPIO_Port, cc1101_SO_Pin) != GPIO_PIN_RESET) {}

#define _MODULE_DESELECT	HAL_GPIO_WritePin(cc1101_CSN_GPIO_Port, cc1101_CSN_Pin, GPIO_PIN_SET);
	
typedef enum _reading_writing_operation_ {
	RW_WRITING = 0x00,
	RW_READING = 0x80
} RW_Operation;

typedef enum _brust_single_operation_ {
	BS_SINGLE = 0x00,
	BS_BRUST = 0x40
} BS_Operation;
	
typedef enum _cc1101_state_ {
	IDLE = 0,
	RX,
	TX,
	CALIBRATE = 4,
	SETTINGS,
	RX_FIFO_OVERFLOW,
	TX_FIFO_UNDERFLOW
} RFmodule_state;

void power_up_reset(void);
uint8_t module_init(void);

void issue_strobe(uint8_t strobe, RW_Operation rw, uint8_t *status);
uint8_t status_register(uint8_t status_register, RW_Operation rw, uint8_t *status);
uint8_t read_register(uint8_t start_address, uint8_t *reg_value, uint8_t size);
uint8_t write_register(uint8_t start_reg_address, uint8_t *reg_value, uint8_t size);
uint8_t write_register_brust(uint8_t start_reg_address, uint8_t *reg_value, uint8_t size);

uint8_t fifo_access(uint8_t *fifo_data, uint8_t size, RW_Operation rw);

#endif
