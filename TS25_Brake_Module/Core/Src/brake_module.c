/*
 * brake_module.c
 *
 *  Created on: Aug 8, 2024
 *      Author: Lexi Boan
 */

#include "brake_module.h"

BrakeModuleInfo_t BrakeModule;

void TransmitHeartBeat()
{
	//Increment counter by 1, if 255 force to 0
	if(BrakeModule.HeartBeatCounter == 255)
	{
		BrakeModule.HeartBeatCounter = 0;
		//BrakeModule.HeartBeatState = !BrakeModule.HeartBeatState;
	}
	else
	{
		BrakeModule.HeartBeatCounter++;
		//BrakeModule.HeartBeatState = !BrakeModule.HeartBeatState;
	}
	//BrakeModule.canHeartBeat.TxData[0] = BrakeModule.HeartBeatState;
	BrakeModule.canHeartBeat.TxData[1] = BrakeModule.HeartBeatCounter;

	//Toggle debug LED
	HAL_GPIO_TogglePin(BrakeModule.dbgLedPort, BrakeModule.dbgLedPin);

	// Adjust to “HAL_FDCAN_AddMessageToTxFifoQ” to match TS25_Temp_Module FDCAN function
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &BrakeModule.canHeartBeat.TxHeader, BrakeModule.canHeartBeat.TxData);

}

void TransmitDigital()
{
	BrakeModule.canDigital.TxData[0] = BrakeModule.HighPressure;
	BrakeModule.canDigital.TxData[1] = BrakeModule.LowPressure;
	BrakeModule.canDigital.TxData[2] = BrakeModule.FiveKW;
	BrakeModule.canDigital.TxData[3] = BrakeModule.BSPD_OK;
	BrakeModule.canDigital.TxData[4] = BrakeModule.BSPDLatch;
	BrakeModule.canDigital.TxData[5] = BrakeModule.GL_InOK;

	// Adjust to “HAL_FDCAN_AddMessageToTxFifoQ” to match TS25_Temp_Module FDCAN function
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &BrakeModule.canDigital.TxHeader, BrakeModule.canDigital.TxData);
}

void UpdateDigital()
{
	if (BrakeModule.LowPressureRaw >= 3300) {
		BrakeModule.LowPressure = 1;
	} else {
		BrakeModule.LowPressure = 0;
	}
	if (BrakeModule.HighPressureRaw >= 3300) {
		BrakeModule.LowPressure = 1;
	} else {
		BrakeModule.LowPressure = 0;
	}
	if (BrakeModule.BSPDLatchRaw >= 3300) {
		BrakeModule.BSPDLatch = 1;
	} else {
		BrakeModule.BSPDLatch = 0;
	}
	if (BrakeModule.GL_InOKRaw >= 3300) {
		BrakeModule.GL_InOK = 1;
	} else {
		BrakeModule.GL_InOK = 0;
	}

	// FiveKW and BSPD_OK
	BrakeModule.FiveKW = HAL_GPIO_ReadPin(BrakeModule.FiveKWInPort, BrakeModule.FiveKWInPin);
	BrakeModule.BSPD_OK = HAL_GPIO_ReadPin(BrakeModule.BSPD_OK_InPort, BrakeModule.BSPD_OK_InPin);
}

// Save the raw data into CAN
void TransmitDigitalRaw()
{
	BrakeModule.canDigitalRaw.TxData[0] = (BrakeModule.LowPressureRaw >> 8);
	BrakeModule.canDigitalRaw.TxData[1] = (BrakeModule.LowPressureRaw & 0xFF);
	BrakeModule.canDigitalRaw.TxData[2] = (BrakeModule.HighPressureRaw >> 8);
	BrakeModule.canDigitalRaw.TxData[3] = (BrakeModule.HighPressureRaw & 0xFF);
	BrakeModule.canDigitalRaw.TxData[4] = (BrakeModule.BSPDLatchRaw >> 8);
	BrakeModule.canDigitalRaw.TxData[5] = (BrakeModule.BSPDLatchRaw & 0xFF);
	BrakeModule.canDigitalRaw.TxData[6] = (BrakeModule.GL_InOKRaw >> 8);
	BrakeModule.canDigitalRaw.TxData[7] = (BrakeModule.GL_InOKRaw & 0xFF);

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &BrakeModule.canDigitalRaw.TxHeader, BrakeModule.canAnalogue.TxData);
}

// get data from real life
void UpdateDigitalRaw()
{
	//???
	uint8_t test[2] = { 0xAA, 0xAA};
	uint8_t testRX[2] = { 0, 0 };

	// Low Pressure
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 0);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.LowPressureRaw = (int)(DIGITAL_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	// High Pressure
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 1);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.HighPressureRaw = (int)(DIGITAL_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	// BSPDLatch
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 6);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.BSPDLatchRaw = (int)(DIGITAL_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	// GL_InOK
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 7);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.GL_InOKRaw = (int)(DIGITAL_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

}

void TransmitAnalogue()
{
	BrakeModule.canAnalogue.TxData[0] = (BrakeModule.Brake1_RawInt >> 8);
	BrakeModule.canAnalogue.TxData[1] = (BrakeModule.Brake1_RawInt & 0xFF);
	BrakeModule.canAnalogue.TxData[2] = (BrakeModule.Brake2_RawInt >> 8);
	BrakeModule.canAnalogue.TxData[3] = (BrakeModule.Brake2_RawInt & 0xFF);
	BrakeModule.canAnalogue.TxData[4] = (BrakeModule.LowRefInt >> 8);
	BrakeModule.canAnalogue.TxData[5] = (BrakeModule.LowRefInt & 0xFF);
	BrakeModule.canAnalogue.TxData[6] = (BrakeModule.HighRef1Int >> 8);
	BrakeModule.canAnalogue.TxData[7] = (BrakeModule.HighRef1Int & 0xFF);

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &BrakeModule.canAnalogue.TxHeader, BrakeModule.canAnalogue.TxData);
}

void UpdateAnalogue() //analogue raw readings kinda whack
{
	//???
	uint8_t test[2] = { 0xAA, 0xAA};
	uint8_t testRX[2] = { 0, 0 };

	//read brake1 input
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 3);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.Brake1_Raw = (BRAKE1_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	//BrakeModule.Brake1_Raw = BrakeModule.Brake1_Raw / (11.67 / (1 - (DEADZONE * 0.01)));
	BrakeModule.Brake1_RawInt = (int)BrakeModule.Brake1_Raw;// * 1000;


	//read brake2 input
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 2);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.Brake2_Raw = (BRAKE2_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	//BrakeModule.Brake2_Raw = BrakeModule.Brake2_Raw / (11.67 / (1 - (DEADZONE * 0.01)));
	BrakeModule.Brake2_RawInt = (int)BrakeModule.Brake2_Raw;

	//read lowref input
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 4);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.LowRef = (LOWREF_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	//BrakeModule.LowRefInt = ((int)BrakeModule.LowRef * 1000);
	//BrakeModule.LowRef = BrakeModule.LowRef * (1000);
	BrakeModule.LowRefInt = (int)BrakeModule.LowRef; //placeholder

	//read highref input
	writeSingleRegister(&BrakeModule.Brake_ADC1, CHANNEL_SEL_ADDRESS, 5);
	spiSendReceiveArray(&BrakeModule.Brake_ADC1, test, testRX, 2);
	BrakeModule.HighRef1 = (HIGHREF_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	//BrakeModule.HighRef1Int = ((int)BrakeModule.HighRef1 * 1000);
	//BrakeModule.HighRef1 = BrakeModule.HighRef1 * (1000);
	BrakeModule.HighRef1Int = (int)BrakeModule.HighRef1; //placeholder
}

void canFramesDefine()
{
	//HeartBeat
	BrakeModule.canHeartBeat.canPeripheral = &hfdcan2;
	BrakeModule.canHeartBeat.TxHeader.IdType = FDCAN_STANDARD_ID;
	BrakeModule.canHeartBeat.TxHeader.Identifier = 0x150;
	BrakeModule.canHeartBeat.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	BrakeModule.canHeartBeat.TxHeader.DataLength = 0x02;
	BrakeModule.canHeartBeat.TxData[0] = 0;
	BrakeModule.canHeartBeat.TxData[1] = 0;

	//Digital
	BrakeModule.canDigital.canPeripheral = &hfdcan2;
	BrakeModule.canDigital.TxHeader.IdType = FDCAN_STANDARD_ID;
	BrakeModule.canDigital.TxHeader.Identifier = 0x152;
	BrakeModule.canDigital.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	BrakeModule.canDigital.TxHeader.DataLength = 0x06;
	BrakeModule.canDigital.TxData[0] = 0;
	BrakeModule.canDigital.TxData[1] = 0;
	BrakeModule.canDigital.TxData[2] = 0;
	BrakeModule.canDigital.TxData[3] = 0;
	BrakeModule.canDigital.TxData[4] = 0;
	BrakeModule.canDigital.TxData[5] = 0;

	//Digital RAW
	BrakeModule.canAnalogue.canPeripheral = &hfdcan2;
	BrakeModule.canAnalogue.TxHeader.IdType = FDCAN_STANDARD_ID;
	BrakeModule.canAnalogue.TxHeader.Identifier = 0x153;
	BrakeModule.canAnalogue.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	BrakeModule.canAnalogue.TxHeader.DataLength = 0x08;
	BrakeModule.canAnalogue.TxData[0] = 0;
	BrakeModule.canAnalogue.TxData[1] = 0;
	BrakeModule.canAnalogue.TxData[2] = 0;
	BrakeModule.canAnalogue.TxData[3] = 0;
	BrakeModule.canAnalogue.TxData[4] = 0;
	BrakeModule.canAnalogue.TxData[5] = 0;
	BrakeModule.canAnalogue.TxData[6] = 0;
	BrakeModule.canAnalogue.TxData[7] = 0;

	//Analogue
	BrakeModule.canAnalogue.canPeripheral = &hfdcan2;
	BrakeModule.canAnalogue.TxHeader.IdType = FDCAN_STANDARD_ID;
	BrakeModule.canAnalogue.TxHeader.Identifier = 0x154;
	BrakeModule.canAnalogue.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	BrakeModule.canAnalogue.TxHeader.DataLength = 0x08;
	BrakeModule.canAnalogue.TxData[0] = 0;
	BrakeModule.canAnalogue.TxData[1] = 0;
	BrakeModule.canAnalogue.TxData[2] = 0;
	BrakeModule.canAnalogue.TxData[3] = 0;
	BrakeModule.canAnalogue.TxData[4] = 0;
	BrakeModule.canAnalogue.TxData[5] = 0;
	BrakeModule.canAnalogue.TxData[6] = 0;
	BrakeModule.canAnalogue.TxData[7] = 0;
}

void ioAssign()
{
	//anything else needed here?
	//SPI
	BrakeModule.Brake_ADC1.SPI_Handle = &hspi1;
	BrakeModule.Brake_ADC1.CS_Port = GPIOA;
	BrakeModule.Brake_ADC1.CS_Pin = GPIO_PIN_4;

	//outputs
	BrakeModule.dbgLedPort = GPIOB;
	BrakeModule.dbgLedPin = GPIO_PIN_5;
	HAL_GPIO_WritePin(BrakeModule.dbgLedPort, BrakeModule.dbgLedPin, GPIO_PIN_RESET); //LED on (default)

	//inputs
	/*
	BrakeModule.LowPressureInPort = GPIOB;
	BrakeModule.LowPressureInPin = GPIO_PIN_2;

	BrakeModule.HighPressureInPort = GPIOB;
	BrakeModule.HighPressureInPin = GPIO_PIN_1;

	BrakeModule.BSPDLatchInPort = GPIOA;
	BrakeModule.BSPDLatchInPin = GPIO_PIN_3;

	BrakeModule.GL_InOK_InPort = GPIOA;
	BrakeModule.GL_InOK_InPin = GPIO_PIN_8;
*/

	BrakeModule.BSPD_OK_InPort = GPIOB;
	BrakeModule.BSPD_OK_InPin = GPIO_PIN_2;

	BrakeModule.FiveKWInPort = GPIOA;
	BrakeModule.FiveKWInPin = GPIO_PIN_5;
}

void initialiseADC()
{
	HAL_Delay(50);

	initADS7028(&BrakeModule.Brake_ADC1);

	HAL_Delay(1000);
}
