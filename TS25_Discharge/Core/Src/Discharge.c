/*
 * Discharge.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Thomas Orr
 */

#include "Discharge.h"

DischargeInfo_t Discharge;
unit16_t ThermistorResistance;

void TrasmitHeartBeat()
{
	//Increment counter by 1, if 255 forced to 0
	if(Discharge.HeartBeatCounter == 255)
	{
		Discharge.HeartBeatCounter = 0;
	}
	else
	{
		Discharge.HeartBeatCounter++;
	}
	Discharge.canHeartBeat.TxData[1] = Discharge.HeartBeatCounter;

	//Toggle debug LED
	HAL_GPIO_TogglePin(Discharge.dbgLedPort, Discharge.dbg.LedPin);

	Discharge.canHeartBeat.transmitFlag = 1;
}

void TransmitDigital()
{
	Discharge.canDigital.TxData[0] = Discharge.DischargeDisable;
	Discharge.canDigital.TxData[1] = Discharge.FiveKW;
	Discharge.canDigital.TxData[2] = Discharge.PDOC_ok;

	//HAL_CAN_AddTxMessage(&hcan, &Discharge.canDigital.TxHeader, Discharge.canDigital.TxData, &Discharge.canDigital.TxMailbox);
}

void UpdateDigital()
{
	Discharge.DischargeDisable = HAL_GPIO_ReadPin(Discharge.DischargeDisableInPort, Discharge.DischargeDisableInPin);
	Discharge.FiveKW = HAL_GPIO_ReadPin(Discharge.FiveKWInPort, Discharge.FiveKWInPin);
	Discharge.PDOC_ok = HAL_GPIO_ReadPin(Discharge.PDOCokInPort, Discharge.PDOCokInPin);
}

/**
 * Sets flag to enable the transmission of the Analogue frames.
**/
void TransmitAnalogue()
{
	Discharge.canAnalogue.TxData[0] = (Discharge.MC_Voltage >> 8);
		Discharge.canAnalogue.TxData[1] = (Discharge.MC_Voltage & 0xFF);
		Discharge.canAnalogue.TxData[2] = (Discharge.HV_Active_Voltage >> 8);
		Discharge.canAnalogue.TxData[3] = (Discharge.HV_Active_Voltage & 0xFF);
		Discharge.canAnalogue.TxData[4] = (Discharge.PDOC_Sensor_Temp >> 8);
		Discharge.canAnalogue.TxData[5] = (Discharge.PDOC_Sensor_Temp & 0xFF);
		Discharge.canAnalogue.TxData[6] = (Discharge.PDOC_Ref_Temp >> 8);
		Discharge.canAnalogue.TxData[7] = (Discharge.PDOC_Ref_Temp & 0xFF);

		//HAL_CAN_AddTxMessage(&hcan, &Discharge.canAnalogue.TxHeader, Discharge.canAnalogue.TxData, &Discharge.canAnalogue.TxMailbox);
}

/**
 * Sets flag to enable the transmission of the Analogue RAW frames.
**/
void TransmitAnalogueRaw()
{
	Discharge.canAnalogueRaw.TxData[0] = (Discharge.HV_Sense_Raw_Voltage >> 8);
		Discharge.canAnalogueRaw.TxData[1] = (Discharge.HV_Sense_Raw_Voltage & 0xFF);
		Discharge.canAnalogueRaw.TxData[2] = (Discharge.HV_Sense_Ref_Raw_Voltage >> 8);
		Discharge.canAnalogueRaw.TxData[3] = (Discharge.HV_Sense_Ref_Raw_Voltage & 0xFF);
		Discharge.canAnalogueRaw.TxData[4] = (Discharge.PDOC_Sensor_Raw_Voltage >> 8);
		Discharge.canAnalogueRaw.TxData[5] = (Discharge.PDOC_Sensor_Raw_Voltage & 0xFF);
		Discharge.canAnalogueRaw.TxData[6] = (Discharge.PDOC_Ref_Raw_Voltage >> 8);
		Discharge.canAnalogueRaw.TxData[7] = (Discharge.PDOC_Ref_Raw_Voltage & 0xFF);

		//HAL_CAN_AddTxMessage(&hcan, &Discharge.canAnalogueRaw.TxHeader, Discharge.canAnalogueRaw.TxData, &Discharge.canAnalogueRaw.TxMailbox);
	}

void UpdateAnalogue()
{
	uint8_t test[2] = { 0xAA, 0xAA};
	uint8_t testRX[2] = { 0, 0 };

	//Read HV sense input
	writeSingleRegister(&Discharge.HVSense_ADC, CHANNEL_SEL_ADDRESS, 6);
	spiSendReceiveArray(&Discharge.HVSense_ADC, test, testRX, 2);
	Discharge.HV_Sense_Raw_Voltage = (HV_SENSE_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	Discharge.MC_Voltage = ((HV_R1+HV_R2)/(HV_R2))*(Discharge.HV_Sense_Raw_Voltage/1000.0)*10;

	//Read HV sense input
	writeSingleRegister(&Discharge.HVSense_ADC, CHANNEL_SEL_ADDRESS, 7);
	spiSendReceiveArray(&Discharge.HVSense_ADC, test, testRX, 2);
	Discharge.HV_Sense_Ref_Raw_Voltage = (HV_SENSE_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	Discharge.HV_Active_Voltage = ((HV_R1+HV_R2)/(HV_R2))*(Discharge.HV_Sense_Ref_Raw_Voltage/1000.0)*10;

	//Read PDOC sense input
	writeSingleRegister(&Discharge.PDOC_ADC, CHANNEL_SEL_ADDRESS, 2);
	spiSendReceiveArray(&Discharge.PDOC_ADC, test, testRX, 2);
	Discharge.PDOC_Sensor_Raw_Voltage = (PDOC_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	//Converts thermistor measurement to temperature
	ThermistorResistance = ((Discharge.PDOC_Sensor_Raw_Voltage)/(PDOC_VDD - Discharge.PDOC_Sensor_Raw_Voltage))*(THERM_BIAS_RESIS);
	float term1 = ( 1.0 )/( 298.15 );
	float term2 = (( 1.0 )/( THERMISTOR_BETA ))*log( (ThermistorResistance) / (10000.0) );
	Discharge.PDOC_Sensor_Temp = 10*(( ( 1.0 ) / (term1 + term2)) - 273.15);

	//Read PDOC REF sense input
	writeSingleRegister(&Discharge.PDOC_ADC, CHANNEL_SEL_ADDRESS, 5);
	spiSendReceiveArray(&Discharge.PDOC_ADC, test, testRX, 2);
	Discharge.PDOC_Ref_Raw_Voltage = (PDOC_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	//Converts thermistor measurement to temperature
	ThermistorResistance = ((Discharge.PDOC_Ref_Raw_Voltage)/(PDOC_VDD - Discharge.PDOC_Ref_Raw_Voltage))*(THERM_BIAS_RESIS);
	term1 = ( 1.0 )/( 298.15 );
	term2 = (( 1.0 )/( THERMISTOR_BETA ))*log( (ThermistorResistance) / (10000.0) );
	Discharge.PDOC_Ref_Temp = 10*(( ( 1.0 ) / (term1 + term2)) - 273.15);
}

void canFramesDefine()
{
	//Heartbeat
	Discharge.canHeartBeat.canPeripheral = hcan;
	Discharge.canHeartBeat.TxHeader.IDE = CAN_ID_STD;
	Discharge.canHeartBeat.TxHeader.StdId = 0x500;
	Discharge.canHeartBeat.TxHeader.RTR = CAN_RTR_DATA;
	Discharge.canHeartBeat.TxHeader.DLC = 0x02;
	Discharge.canHeartBeat.TxData[0] = 0;
	Discharge.canHeartBeat.TxData[1] = 0;

	//Digital
	Discharge.canDigital.canPeripheral = hcan;
	Discharge.canDigital.TxHeader.IDE = CAN_ID_STD;
	Discharge.canDigital.TxHeader.StdId = 0x453;
	Discharge.canDigital.TxHeader.RTR = CAN_RTR_DATA;
	Discharge.canDigital.TxHeader.DLC = 0x03;
	Discharge.canDigital.TxData[0] = 0;
	Discharge.canDigital.TxData[1] = 0;
	Discharge.canDigital.TxData[2] = 0;

	//Analogue RAW
	Discharge.canAnalogue.canPeripheral = hcan;
	Discharge.canAnalogue.TxHeader.IDE = CAN_ID_STD;
	Discharge.canAnalogue.TxHeader.StdId = 0x504;
	Discharge.canAnalogue.TxHeader.RTR = CAN_RTR_DATA;
	Discharge.canAnalogue.TxHeader.DLC = 0x08;
	Discharge.canAnalogue.TxData[0] = 0;
	Discharge.canAnalogue.TxData[1] = 0;
	Discharge.canAnalogue.TxData[2] = 0;
	Discharge.canAnalogue.TxData[3] = 0;
	Discharge.canAnalogue.TxData[4] = 0;
	Discharge.canAnalogue.TxData[5] = 0;
	Discharge.canAnalogue.TxData[6] = 0;
	Discharge.canAnalogue.TxData[7] = 0;

	//Analogue RAW
	Discharge.canAnalogueRaw.canPeripheral = hcan;
	Discharge.canAnalogueRaw.TxHeader.IDE = CAN_ID_STD;
	Discharge.canAnalogueRaw.TxHeader.StdId = 0x505;
	Discharge.canAnalogueRaw.TxHeader.RTR = CAN_RTR_DATA;
	Discharge.canAnalogueRaw.TxHeader.DLC = 0x08;
	Discharge.canAnalogueRaw.TxData[0] = 0;
	Discharge.canAnalogueRaw.TxData[1] = 0;
	Discharge.canAnalogueRaw.TxData[2] = 0;
	Discharge.canAnalogueRaw.TxData[3] = 0;
	Discharge.canAnalogueRaw.TxData[4] = 0;
	Discharge.canAnalogueRaw.TxData[5] = 0;
	Discharge.canAnalogueRaw.TxData[6] = 0;
	Discharge.canAnalogueRaw.TxData[7] = 0;
}

void ioAssign()
{
	//SPI
	Discharge.HVSense_ADC.SPI_Handle = &hspi1;
	Discharge.HVSense_ADC.CS_Port = GPIOA;
	Discharge.HVSense_ADC.CS_Pin = GPIO_PIN_2;

	Discharge.PDOC_ADC.SPI_Handle = &hspi1;
	Discharge.PDOC_ADC.CS_Port = GPIOB;
	Discharge.PDOC_ADC.CS_Pin = GPIO_PIN_6;

	//Outputs
	Discharge.dbgLedPort = GPIOA;
	Discharge.dbgLedPin = GPIO_PIN_3;
	HAL_GPIO_WritePin(Discharge.dbgLedPort, Discharge.dbgLedPin, GPIO_PIN_RESET); //LED on (default)

	//CS set to high (default state)
	HAL_GPIO_WritePin(Discharge.HVSense_ADC.CS_Port, Discharge.HVSense_ADC.CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Discharge.PDOC_ADC.CS_Port, Discharge.PDOC_ADC.CS_Pin, GPIO_PIN_SET);

	//Inputs
	Discharge.DischargeDisableInPort = GPIOA;
	Discharge.DischargeDisableInPin = GPIO_PIN_7;

	Discharge.FiveKWInPort = GPIOA;
	Discharge.FiveKWInPin = GPIO_PIN_6;

	Discharge.PDOCokInPort = GPIOA;
	Discharge.PDOCokInPin = GPIO_PIN_15;
}

void initialiseADC()
{
	HAL_Delay(50);

	initADS7028(&Discharge.HVSense_ADC);
	initADS7028(&Discharge.PDOC_ADC);

	HAL_Delay(1000);
}
