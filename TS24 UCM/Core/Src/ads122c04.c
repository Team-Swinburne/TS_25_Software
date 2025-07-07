/*
 * ads122c04.c
 *
 *  Created on: Oct 2, 2024
 *      Author: Ethan Jones
 */

#include "ads122c04.h"

ADS122C04Reg_t ADS122C04_DefaultConfig;

static I2C_HandleTypeDef* ADS122C04_24FC_I2C_Handle; //i2c handle

void ADS122C04_Init(I2C_HandleTypeDef *Handle)
{
	//Create default config
	ADS122C04_DefaultConfig.reg0.bit.GAIN = 0;
	ADS122C04_DefaultConfig.reg0.bit.MUX = 0;
	ADS122C04_DefaultConfig.reg0.bit.PGA_BYPASS = 0;

	ADS122C04_DefaultConfig.reg1.bit.DR = 0;
	ADS122C04_DefaultConfig.reg1.bit.MODE = 0;
	ADS122C04_DefaultConfig.reg1.bit.CMBIT = 0;
	ADS122C04_DefaultConfig.reg1.bit.VREF = 0;
	ADS122C04_DefaultConfig.reg1.bit.TS = 0;

	ADS122C04_DefaultConfig.reg2.bit.DRDY = 0;
	ADS122C04_DefaultConfig.reg2.bit.DCNT = 0;
	ADS122C04_DefaultConfig.reg2.bit.CRCbits = 0;
	ADS122C04_DefaultConfig.reg2.bit.BCS = 0;
	ADS122C04_DefaultConfig.reg2.bit.IDAC = 0;

	ADS122C04_DefaultConfig.reg3.bit.I1MUX = 0;
	ADS122C04_DefaultConfig.reg3.bit.I2MUX = 0;
	ADS122C04_DefaultConfig.reg3.bit.RESERVED = 0;

	ADS122C04_24FC_I2C_Handle = Handle;
}

//Framework neutral I2C write
static void ADS122C04_I2C_Write(uint8_t *DataBuffer, uint8_t i2cAddress, uint8_t DataLength)
{
	HAL_I2C_Master_Transmit(ADS122C04_24FC_I2C_Handle, i2cAddress, DataBuffer, DataLength, ADS122C04_I2C_TIMEOUT);
}

//Framework neutral I2C read
static void ADS122C04_I2C_Read(uint8_t *DataBuffer, uint8_t i2cAddress, uint8_t DataLength)
{
	HAL_I2C_Master_Receive(ADS122C04_24FC_I2C_Handle, i2cAddress | 0x01, DataBuffer, DataLength, ADS122C04_I2C_TIMEOUT);
}

void ADS122C04_WriteStartSync()
{
	uint8_t DataBuffer;

	uint8_t address = 0x80;

	DataBuffer = ADS122C04_START_CMD;

	ADS122C04_I2C_Write(&DataBuffer, address, 1);
}

uint32_t ADS122C04_ReadData()
{
	uint8_t DataBuffer;
	uint8_t Response[3];

	uint8_t address = 0x80;

	DataBuffer = ADS122C04_RDATA_CMD;

	ADS122C04_I2C_Write(&DataBuffer, address, 1);

	ADS122C04_I2C_Read(&Response, address, 3);

	return (Response[0] << 16) + (Response[1] << 8) + (Response[2] << 0);
}

uint8_t ADS122C04_ReadRegister(uint8_t RegisterAddress)
{
	uint8_t DataBuffer;
	uint8_t Response;

	uint8_t address = 0x80;

	DataBuffer = ADS122C04_RREG_CMD | (RegisterAddress << 2);

	ADS122C04_I2C_Write(&DataBuffer, address, 1);

	ADS122C04_I2C_Read(&Response, address, 1);

	return Response;
}

void ADS122C04_WriteRegister(uint8_t RegisterAddress, uint8_t Data)
{
	uint8_t DataBuffer[2];

	uint8_t address = 0x80;

	DataBuffer[0] = ADS122C04_WREG_CMD | (RegisterAddress << 2);

	DataBuffer[1] = Data;

	ADS122C04_I2C_Write(DataBuffer, address, 2);
}

void ADS122C04_Configurate(ADS122C04Reg_t *Config)
{
	ADS122C04_WriteRegister(ADS122C04_CONFIG_0_REG, Config->reg0.all);
	ADS122C04_WriteRegister(ADS122C04_CONFIG_1_REG, Config->reg1.all);
	ADS122C04_WriteRegister(ADS122C04_CONFIG_2_REG, Config->reg2.all);
	ADS122C04_WriteRegister(ADS122C04_CONFIG_3_REG, Config->reg3.all);
}

/*
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
*/
