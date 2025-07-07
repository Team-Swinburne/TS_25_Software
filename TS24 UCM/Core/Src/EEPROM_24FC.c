/*
 * EEPROM_24FC.c
 *
 *  Created on: Aug 7, 2024
 *      Author: Ethan Jones
 */

#include "EEPROM_24FC.h"

static I2C_HandleTypeDef* EEPROM_24FC_I2C_Handle; //i2c handle

void EEPROM_24FC_Init(I2C_HandleTypeDef *Handle)
{
	EEPROM_24FC_I2C_Handle = Handle;
}

//Framework neutral I2C write
static void EEPROM_24FC_I2C_Write(uint8_t *DataBuffer, uint8_t i2cAddress, uint8_t DataLength)
{
	HAL_I2C_Master_Transmit(EEPROM_24FC_I2C_Handle, i2cAddress, DataBuffer, DataLength, EEPROM_24FC_I2C_TIMEOUT);
}

//Framework neutral I2C read
static void EEPROM_24FC_I2C_Read(uint8_t *DataBuffer, uint8_t i2cAddress, uint8_t DataLength)
{
	HAL_I2C_Master_Receive(EEPROM_24FC_I2C_Handle, i2cAddress | 0x01, DataBuffer, DataLength, EEPROM_24FC_I2C_TIMEOUT);
}

_Bool EEPROM_24FC_Poll()
{
	if(HAL_I2C_IsDeviceReady(EEPROM_24FC_I2C_Handle, 0xA0, 1, EEPROM_24FC_I2C_TIMEOUT) == HAL_OK) return 1;
	return 0;
}

void EEPROM_24FC_ByteWrite(uint16_t WordAddress, uint8_t Data)
{
	uint8_t DataBuffer[2];

	uint8_t Address = (0xA0);  //Control Code
	Address += (WordAddress/256);  //Block select

	DataBuffer[0] = (WordAddress % 256);
	DataBuffer[1] = Data;

	EEPROM_24FC_I2C_Write(DataBuffer, Address, 2);

	while(!EEPROM_24FC_Poll()); //Wait until EEPROM has finished writing.
}

void EEPROM_24FC_PageWrite(uint16_t WordAddress, uint8_t* Data[], uint8_t Length)
{
	uint8_t Address = (0xA0);  //Control Code
	Address += (WordAddress/256);  //Block select

	//DataBuffer[0] = (WordAddress % 256);

	EEPROM_24FC_I2C_Write(&Data, Address, (Length + 1));

	while(!EEPROM_24FC_Poll()); //Wait until EEPROM has finished writing.
}

uint8_t EEPROM_24FC_ByteRead(uint16_t WordAddress)
{
	uint8_t Address = (0xA0);  //Control Code
	Address += (WordAddress/256);  //Block select
	uint8_t DataBuffer = (WordAddress % 256);

	EEPROM_24FC_I2C_Write(&DataBuffer, Address, 1); //Selects address
	EEPROM_24FC_I2C_Read(&DataBuffer, Address, 1); //Reads address

	return DataBuffer;
}
