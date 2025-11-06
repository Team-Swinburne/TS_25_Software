/*
 * TempModule.c
 *
 *  Created on: Jul 4, 2025
 *      Author: Lexi Boan
 *
 *  V1 - Heartbeat Code: Jul 4, 2025
 *  V2 - CAN Configuration: Jul 7, 2025
 */

#include "TempModule.h"

TempModuleInfo_t TempModule;

/**
 * Converts voltage read to temperature based on the datasheet
 * from the Energus module.
**/
static int8_t EnergusConvertToTemp(uint16_t Voltage)
{
	//Force voltage to datasheet limits, if outside.
	if(Voltage > 2440) { Voltage = 2440; }
	if(Voltage < 1300) { Voltage = 1300; }

	return (-(225.7*pow(Voltage/1000.0, 3)) + (1310.6*pow(Voltage/1000.0, 2)) - (2594.8*pow(Voltage/1000.0, 1)) + 1767.8);
}

/**
 * Handles the heartbeat code, and
 * Sets flag to enable the transmission of the heartbeat frame.
**/
void TransmitHeartBeat()
{
	//Increment counter by 1, if 255 force to 0
	if(TempModule.HeartBeatCounter == 255)
	{
		TempModule.HeartBeatCounter = 0;
	}
	else
	{
		TempModule.HeartBeatCounter++;
	}
	TempModule.canHeartBeat.TxData[1] = TempModule.HeartBeatCounter;

	//Toggle debug LED
	HAL_GPIO_TogglePin(TempModule.dbgLedPort, TempModule.dbgLedPin);

	TempModule.canHeartBeat.transmitFlag = 1;
}

/**
 * Sets flag to enable the transmission of the Analogue frames.
**/
void TransmitAnalogue()
{
	//For each tempbank
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		TempModule.canAnalogue[i].transmitFlag = 1;	 //TB 1 - 6
	}
}

/**
 * Sets flag to enable the transmission of the Analogue RAW frames.
**/
void TransmitAnalogueRaw()
{
	//For each tempbank
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		TempModule.canAnalogueRaw[2*i].transmitFlag = 1;	 //CH 0 - 3
		TempModule.canAnalogueRaw[2*i + 1].transmitFlag = 1; //CH 4 - 7
	}
}

/**
 * Handles transmitting the CAN frames. Function ran continuously in main loop.
 * Checks if CAN Tx mailbox is free and if frame needs to be sent.
**/
void TransmitCAN()
{
	//Heartbeat
	if((HAL_FDCAN_GetTxFifoFreeLevel(TempModule.canHeartBeat.canPeripheral) == 3) && (TempModule.canHeartBeat.transmitFlag == 1))
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TempModule.canHeartBeat.TxHeader, TempModule.canHeartBeat.TxData);
		TempModule.canHeartBeat.transmitFlag = 0;
	}

	//Analogue frames
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		if((HAL_FDCAN_GetTxFifoFreeLevel(TempModule.canAnalogue[i].canPeripheral) == 3) && (TempModule.canAnalogue[i].transmitFlag == 1))
		{
			CalculateTemperature();

			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TempModule.canAnalogue[i].TxHeader, TempModule.canAnalogue[i].TxData);
			TempModule.canAnalogue[i].transmitFlag = 0;
		}
	}

	//Analogue RAW frames
	for(uint8_t i = 0; i < 2*NUM_OF_TEMPBANKS; i++)
	{
		if(TempModule.AverageFlag == 1)
		{
			CalculateVoltageReading();
		}

		if((HAL_FDCAN_GetTxFifoFreeLevel(TempModule.canAnalogueRaw[i].canPeripheral) == 3) && (TempModule.canAnalogueRaw[i].transmitFlag == 1))
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TempModule.canAnalogueRaw[i].TxHeader, TempModule.canAnalogueRaw[i].TxData);
			TempModule.canAnalogueRaw[i].transmitFlag = 0;
		}
	}

	TempModule.AverageFlag = 1;
}

void CalculateVoltageReading()
{
	//Read voltage from all TempBanks
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		for(uint8_t j = 0; j < 8 ; j++)
		{
			//Compute voltage based on average readings
			TempModule.TempBanks[i].RawVoltages[j] = (TempModule.TempBanks[i].RawVoltagesSum[j])/(TempModule.TempBanks[i].RawVoltageCount[j]);

			//Reset to compute next average
			TempModule.TempBanks[i].RawVoltagesSum[j] = 0;
			TempModule.TempBanks[i].RawVoltageCount[j] = 0;
		}

		//Package all RAW voltages to CAN frames
		for(uint8_t j = 0 ; j < 4; j++)
		{
			//Channels 0 - 3
			TempModule.canAnalogueRaw[2*i].TxData[2*j] = (TempModule.TempBanks[i].RawVoltages[j] >> 8);
			TempModule.canAnalogueRaw[2*i].TxData[2*j+1] = (TempModule.TempBanks[i].RawVoltages[j] & 0xFF);

			//Channels 4 - 7
			TempModule.canAnalogueRaw[2*i + 1].TxData[2*j] = (TempModule.TempBanks[i].RawVoltages[j + 4] >> 8);
			TempModule.canAnalogueRaw[2*i + 1].TxData[2*j+1] = (TempModule.TempBanks[i].RawVoltages[j + 4] & 0xFF);
		}
	}

	TempModule.AverageFlag = 0;
}

void CalculateTemperature()
{
	//Read voltage from all TempBanks
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		for(uint8_t j = 0; j< 8 ; j++)
		{
			TempModule.TempBanks[i].Temperature[j] = EnergusConvertToTemp(TempModule.TempBanks[i].RawVoltages[j]);
			TempModule.canAnalogue[i].TxData[j] = TempModule.TempBanks[i].Temperature[j];
		}
	}
}

void UpdateAnalogue()
{
	uint16_t TempBankVDD[6] = {TB1_VDD, TB2_VDD, TB3_VDD, TB4_VDD, TB5_VDD};
	uint8_t test[2] = { 0xAA, 0xAA};
	uint8_t testRX[2] = { 0, 0 };

	//Read voltage from all TempBanks
	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		for(uint8_t j = 0; j< 8 ; j++)
		{
			writeSingleRegister(&TempModule.ADC_Bank[i], CHANNEL_SEL_ADDRESS, j);
			spiSendReceiveArray(&TempModule.ADC_Bank[i], test, testRX, 2);
			TempModule.TempBanks[i].RawVoltagesSum[j] += (TempBankVDD[i]*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
			TempModule.TempBanks[i].RawVoltageCount[j]++;
		}
	}
}

void canFramesDefine()
{

	//Heartbeat
	TempModule.canHeartBeat.canPeripheral = &hfdcan2;
	TempModule.canHeartBeat.TxHeader.IdType = FDCAN_STANDARD_ID;
	TempModule.canHeartBeat.TxHeader.Identifier = 0x110;
	TempModule.canHeartBeat.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TempModule.canHeartBeat.TxHeader.DataLength = 0x02;
	TempModule.canHeartBeat.TxData[0] = 0;
	TempModule.canHeartBeat.TxData[1] = 0;

	//Analogue
	for(int i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		TempModule.canAnalogue[i].canPeripheral = &hfdcan2;
		TempModule.canAnalogue[i].TxHeader.IdType = FDCAN_STANDARD_ID;
		TempModule.canAnalogue[i].TxHeader.Identifier = 0x120 + i;
		TempModule.canAnalogue[i].TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TempModule.canAnalogue[i].TxHeader.DataLength = 0x08;
		TempModule.canAnalogue[i].TxData[0] = 0;
		TempModule.canAnalogue[i].TxData[1] = 0;
		TempModule.canAnalogue[i].TxData[2] = 0;
		TempModule.canAnalogue[i].TxData[3] = 0;
		TempModule.canAnalogue[i].TxData[4] = 0;
		TempModule.canAnalogue[i].TxData[5] = 0;
		TempModule.canAnalogue[i].TxData[6] = 0;
		TempModule.canAnalogue[i].TxData[7] = 0;
	}

	//Analogue RAW
	for(int i = 0; i < 2*NUM_OF_TEMPBANKS; i++)
	{
		TempModule.canAnalogueRaw[i].canPeripheral = &hfdcan2;
		TempModule.canAnalogueRaw[i].TxHeader.IdType = FDCAN_STANDARD_ID;
		TempModule.canAnalogueRaw[i].TxHeader.Identifier = 0x130 + i;
		TempModule.canAnalogueRaw[i].TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TempModule.canAnalogueRaw[i].TxHeader.DataLength = 0x08;
		TempModule.canAnalogueRaw[i].TxData[0] = 0;
		TempModule.canAnalogueRaw[i].TxData[1] = 0;
		TempModule.canAnalogueRaw[i].TxData[2] = 0;
		TempModule.canAnalogueRaw[i].TxData[3] = 0;
		TempModule.canAnalogueRaw[i].TxData[4] = 0;
		TempModule.canAnalogueRaw[i].TxData[5] = 0;
		TempModule.canAnalogueRaw[i].TxData[6] = 0;
		TempModule.canAnalogueRaw[i].TxData[7] = 0;
	}
}

void ioAssign()
{
	//Temp Bank 1
	TempModule.ADC_Bank[0].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[0].CS_Port = GPIOB;
	TempModule.ADC_Bank[0].CS_Pin = GPIO_PIN_2;

	//Temp Bank 2
	TempModule.ADC_Bank[1].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[1].CS_Port = GPIOA;
	TempModule.ADC_Bank[1].CS_Pin = GPIO_PIN_8;

	//Temp Bank 3
	TempModule.ADC_Bank[2].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[2].CS_Port = GPIOA;
	TempModule.ADC_Bank[2].CS_Pin = GPIO_PIN_9;

	//Temp Bank 4
	TempModule.ADC_Bank[3].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[3].CS_Port = GPIOC;
	TempModule.ADC_Bank[3].CS_Pin = GPIO_PIN_6;

	//Temp Bank 5
	TempModule.ADC_Bank[4].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[4].CS_Port = GPIOA;
	TempModule.ADC_Bank[4].CS_Pin = GPIO_PIN_10;

	//Temp Bank 6 (AUX Bank)
	/*TempModule.ADC_Bank[5].SPI_Handle = &hspi1;
	TempModule.ADC_Bank[5].CS_Port = GPIOB;
	TempModule.ADC_Bank[5].CS_Pin = GPIO_PIN_6;
	*/

	//Outputs
	TempModule.dbgLedPort = GPIOA;
	TempModule.dbgLedPin = GPIO_PIN_5;
	HAL_GPIO_WritePin(TempModule.dbgLedPort, TempModule.dbgLedPin, GPIO_PIN_RESET); //LED on (default)
}

void initialiseADC()
{
	HAL_Delay(50);

	for(uint8_t i = 0; i < NUM_OF_TEMPBANKS; i++)
	{
		initADS7028(&TempModule.ADC_Bank[i]);
		HAL_Delay(50);
	}

	HAL_Delay(1000);
}
