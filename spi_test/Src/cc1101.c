#include "cc1101.h"

extern SPI_HandleTypeDef hspi1;

uint8_t halRfWriteReg(uint8_t reg, uint8_t data)
{
	uint8_t d = data;
	return write_register(reg, &d, 1);
}

uint8_t module_init(void)
{
	
//
// Rf settings for CC1101
//
halRfWriteReg(IOCFG2,0x01);  //GDO2 Output Pin Configuration
halRfWriteReg(IOCFG0,0x06);  //GDO0 Output Pin Configuration
halRfWriteReg(FIFOTHR,0x47); //RX FIFO and TX FIFO Thresholds
halRfWriteReg(PKTLEN,0x05);  //Packet Length
halRfWriteReg(PKTCTRL1,0x4C);//Packet Automation Control
halRfWriteReg(PKTCTRL0,0x44);//Packet Automation Control
halRfWriteReg(FSCTRL1,0x06); //Frequency Synthesizer Control
halRfWriteReg(FREQ2,0x21);   //Frequency Control Word, High Byte
halRfWriteReg(FREQ1,0x62);   //Frequency Control Word, Middle Byte
halRfWriteReg(FREQ0,0x76);   //Frequency Control Word, Low Byte
halRfWriteReg(MDMCFG4,0xF5); //Modem Configuration
halRfWriteReg(MDMCFG3,0x83); //Modem Configuration
halRfWriteReg(MDMCFG2,0x13); //Modem Configuration
halRfWriteReg(MDMCFG1,0xA2); //Modem Configuration
halRfWriteReg(DEVIATN,0x15); //Modem Deviation Setting
halRfWriteReg(MCSM1,0x3E);   //Main Radio Control State Machine Configuration
halRfWriteReg(MCSM0,0x18);   //Main Radio Control State Machine Configuration
halRfWriteReg(FOCCFG,0x16);  //Frequency Offset Compensation Configuration
halRfWriteReg(WORCTRL,0xFB); //Wake On Radio Control
halRfWriteReg(FSCAL3,0xE9);  //Frequency Synthesizer Calibration
halRfWriteReg(FSCAL2,0x2A);  //Frequency Synthesizer Calibration
halRfWriteReg(FSCAL1,0x00);  //Frequency Synthesizer Calibration
halRfWriteReg(FSCAL0,0x1F);  //Frequency Synthesizer Calibration
halRfWriteReg(TEST2,0x81);   //Various Test Settings
halRfWriteReg(TEST1,0x35);   //Various Test Settings
halRfWriteReg(TEST0,0x09);   //Various Test Settings

return 0;
}

uint8_t status_register(uint8_t status_register, RW_Operation rw, uint8_t *status)
{
	uint8_t result;
	
	status_register |= (rw | BS_BRUST);
	
	_MODULE_SELECT
	WAIT_CHP_RDY
	
	HAL_SPI_TransmitReceive(&hspi1, &status_register, status, 1, 0xFF);
	HAL_SPI_Receive(&hspi1, &result, 1, 0xFF);
	
	WAIT_CHP_RDY
	_MODULE_DESELECT
	
	return result;
}

void issue_strobe(uint8_t strobe, RW_Operation rw, uint8_t *status)
{
	strobe |= ( rw | BS_SINGLE );
	
	_MODULE_SELECT
	WAIT_CHP_RDY
	
	HAL_SPI_TransmitReceive(&hspi1, &strobe, status, 1, 0xFF);
	
	WAIT_CHP_RDY
	_MODULE_DESELECT
}


void power_up_reset(void)
{
	uint8_t status;
	
	_MODULE_SELECT
	HAL_Delay(10);
	_MODULE_DESELECT
	HAL_Delay(100);
	
	issue_strobe(SRES_STROBE, RW_WRITING, &status);
}

uint8_t write_register(uint8_t start_reg_address, uint8_t *reg_value, uint8_t size)
{
	uint8_t status, tmp;
	
	start_reg_address |= ( RW_WRITING | ( (size == 1) ? BS_SINGLE : BS_BRUST ) );
	
	_MODULE_SELECT
	WAIT_CHP_RDY
	
	HAL_SPI_TransmitReceive(&hspi1, &start_reg_address, &status, 1, 0xFF);
	do{
		WAIT_CHP_RDY
		
		HAL_SPI_TransmitReceive(&hspi1, reg_value++, &tmp, 1, 0xFF);
		
		if(tmp != status){
			_MODULE_DESELECT
			return tmp;
		}
	}while( (--size) > 0 );
	
	_MODULE_DESELECT
	return status;
}

uint8_t write_register_brust(uint8_t start_reg_address, uint8_t *reg_value, uint8_t size)
{
	uint8_t status, tmp;
	
	start_reg_address |= ( RW_WRITING | BS_BRUST );
	
	_MODULE_SELECT
	WAIT_CHP_RDY
	
	HAL_SPI_TransmitReceive(&hspi1, &start_reg_address, &status, 1, 0xFF);
	do{
		WAIT_CHP_RDY
		
		HAL_SPI_TransmitReceive(&hspi1, reg_value++, &tmp, 1, 0xFF);
		
		if(tmp != status){
			_MODULE_DESELECT
			return tmp;
		}
	}while( (--size) > 0 );
	
	_MODULE_DESELECT
	return status;
}

uint8_t read_register(uint8_t start_reg_address, uint8_t *reg_value, uint8_t size)
{
	uint8_t status_word;
	
	start_reg_address |= ( size == 1 ? BS_SINGLE : BS_BRUST | RW_READING );
	
	_MODULE_SELECT
	
	WAIT_CHP_RDY
	
	HAL_SPI_TransmitReceive(&hspi1, &start_reg_address, &status_word, 1, 0xFF);
	
	do{
		
		WAIT_CHP_RDY
		
		HAL_SPI_Receive(&hspi1, reg_value++, 1, 0xFF);
		
	}while ( (--size) > 0 );
	
	_MODULE_DESELECT
	
	return status_word;
}

uint8_t fifo_access(uint8_t *fifo_data, uint8_t size, RW_Operation rw)
{
	uint8_t address = ( FIFO_ACCESS | rw | (size == 1 ? BS_SINGLE : BS_BRUST) );
	uint8_t status, tmp;
	
	_MODULE_SELECT
	WAIT_CHP_RDY
	
	switch(rw)
	{
		case RW_READING :
			HAL_SPI_TransmitReceive(&hspi1, &address, &status, 1, 0xFF);
			HAL_SPI_Receive(&hspi1, fifo_data, size, 0xFF);
			break;
		
		case RW_WRITING :
			HAL_SPI_TransmitReceive(&hspi1, &address, &status, 1, 0xFF);
			if(size == 1)
			{
				HAL_SPI_TransmitReceive(&hspi1, fifo_data, &status, 1, 0xFF);
			}
			else
			{
				do{
					HAL_SPI_TransmitReceive(&hspi1, fifo_data++, &tmp, 1, 0xFF);
					if(tmp != status)
					{
						_MODULE_DESELECT
						return tmp;
					}
				}while( (--size > 0) );
			}
			break;
	}
	
	_MODULE_DESELECT
	return status;
}

