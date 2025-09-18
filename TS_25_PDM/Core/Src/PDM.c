/*
 * PDM.c
 *
 *  Created on: Aug 14, 2025
 *      Author: Lexi Boan
 */

#include "PDM.h"

PDMInfo_t PDM;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void CheckReceivedCAN()
{
	HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData);

	for(uint8_t i = 0; i < 5; i++)
	{
		if ((RxHeader.Identifier == 0x260 + i))
		{
			//Set dutycycle to 100, if received data is above 100.
			for(uint8_t i = 0; i < 2; i++)
			{
				if(RxData[i] > 100)
				{
					RxData[i] = 100;
				}
			}

			PDM.driverSwitch[i].Grab[HSD_A].DutyCycle = 100;//RxData[0];
			PDM.driverSwitch[i].Grab[HSD_B].DutyCycle = RxData[1];

			updatePWM(i);
		}
	}

}

void updatePWM(uint8_t driverSelect)
{
	//A
	TIM2->CCR3 = 31999*(PDM.driverSwitch[driverSelect].Grab[HSD_A].DutyCycle/100.0);

	//B
	TIM2->CCR4 = 31999*(PDM.driverSwitch[driverSelect].Grab[HSD_B].DutyCycle/100.0);
}

void TransmitHeartBeat()
{
	//Increment counter by 1, if 255 forced to 0
	if(PDM.HeartBeatCounter == 255)
	{
		PDM.HeartBeatCounter = 0;
	}
	else
	{
		PDM.HeartBeatCounter++;
	}
	PDM.canHeartBeat.TxData[1] = PDM.HeartBeatCounter;

	//Toggle debug LED
	HAL_GPIO_TogglePin(PDM.dbgLedPort, PDM.dbgLedPin);

	PDM.canHeartBeat.transmitFlag = 1;
}

void TransmitDigital()
{
	PDM.canDigital.TxData[0] = PDM.GL_Active;
	PDM.canDigital.TxData[1] = PDM.BL_Active;

	PDM.canDigital.transmitFlag = 1;
}

void UpdateDigital()
{
	PDM.GL_Active = HAL_GPIO_ReadPin(PDM.GL_ActivePort, PDM.GL_ActivePin);
	PDM.BL_Active = HAL_GPIO_ReadPin(PDM.BL_ActivePort, PDM.BL_ActivePin);
}

void TransmitDriverOut()
{

	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.canDriverOut[i].TxData[0] = (PDM.Drivers[i].OutputA);
		PDM.canDriverOut[i].TxData[1] = (PDM.Drivers[i].OutputB);

		PDM.canDriverOut[i].transmitFlag = 1;
	}
}

void UpdateDriverOut()
{
	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.Drivers[i].OutputA = HAL_GPIO_ReadPin(PDM.Drivers[i].OutputAPort, PDM.Drivers[i].OutputAPin);
		PDM.Drivers[i].OutputB = HAL_GPIO_ReadPin(PDM.Drivers[i].OutputBPort, PDM.Drivers[i].OutputBPin);
	}
}

void TransmitHSDrivers()
{
	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.canDriverDiag[i].TxData[0] = (PDM.driverSwitch[i].Driver[HSD_A].Current >> 8);
		PDM.canDriverDiag[i].TxData[1] = (PDM.driverSwitch[i].Driver[HSD_A].Current & 0xFF);
		PDM.canDriverDiag[i].TxData[2] = (PDM.driverSwitch[i].Driver[HSD_A].DutyCycle);
		PDM.canDriverDiag[i].TxData[3] = (PDM.driverSwitch[i].Driver[HSD_B].Current >> 8);
		PDM.canDriverDiag[i].TxData[4] = (PDM.driverSwitch[i].Driver[HSD_B].Current & 0xFF);
		PDM.canDriverDiag[i].TxData[5] = (PDM.driverSwitch[i].Driver[HSD_B].DutyCycle);

		PDM.canDriverDiag[i].transmitFlag = 1;
	}
}

void UpdateHSDrivers()
{
	uint8_t test[2] = { 0xAA, 0xAA};
	uint8_t testRX[2] = { 0, 0 };

	//Read Current for Driver1
	writeSingleRegister(&PDM.HSD_ADC2, CHANNEL_SEL_ADDRESS, 4);
	spiSendReceiveArray(&PDM.HSD_ADC2, test, testRX, 2);
	PDM.driverSwitch[0].Driver[HSD_A].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[0].Driver[HSD_B].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	//Read Current for Driver2
	writeSingleRegister(&PDM.HSD_ADC2, CHANNEL_SEL_ADDRESS, 5);
	spiSendReceiveArray(&PDM.HSD_ADC2, test, testRX, 2);
	PDM.driverSwitch[1].Driver[HSD_A].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[1].Driver[HSD_A].DutyCycle = PDM.driverSwitch[1].Grab[HSD_A].DutyCycle;
	PDM.driverSwitch[1].Driver[HSD_B].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[1].Driver[HSD_B].DutyCycle = PDM.driverSwitch[1].Grab[HSD_A].DutyCycle;

	//Read Current for Driver3
	writeSingleRegister(&PDM.HSD_ADC1, CHANNEL_SEL_ADDRESS, 6);
	spiSendReceiveArray(&PDM.HSD_ADC1, test, testRX, 2);
	PDM.driverSwitch[2].Driver[HSD_A].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[2].Driver[HSD_B].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));

	//Read Current for Driver4
	writeSingleRegister(&PDM.HSD_ADC1, CHANNEL_SEL_ADDRESS, 7);
	spiSendReceiveArray(&PDM.HSD_ADC1, test, testRX, 2);
	PDM.driverSwitch[3].Driver[HSD_A].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[3].Driver[HSD_A].DutyCycle = PDM.driverSwitch[3].Grab[HSD_A].DutyCycle;
	PDM.driverSwitch[3].Driver[HSD_B].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[3].Driver[HSD_B].DutyCycle = PDM.driverSwitch[3].Grab[HSD_A].DutyCycle;

	//Read Current for Driver5
	writeSingleRegister(&PDM.HSD_ADC1, CHANNEL_SEL_ADDRESS, 1);
	spiSendReceiveArray(&PDM.HSD_ADC1, test, testRX, 2);
	PDM.driverSwitch[4].Driver[HSD_A].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[4].Driver[HSD_A].DutyCycle = PDM.driverSwitch[4].Grab[HSD_A].DutyCycle;
	PDM.driverSwitch[4].Driver[HSD_B].Current = (HSD_VDD*(((256*(testRX[0]) + (testRX[1])) >> 4)/4096.0));
	PDM.driverSwitch[4].Driver[HSD_B].DutyCycle = PDM.driverSwitch[4].Grab[HSD_A].DutyCycle;
}

void TransmitCAN()
{
	if ((HAL_FDCAN_GetTxFifoFreeLevel(PDM.canHeartBeat.canPeripheral) == 3) && (PDM.canHeartBeat.transmitFlag == 1))
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &PDM.canHeartBeat.TxHeader, PDM.canHeartBeat.TxData);
		PDM.canHeartBeat.transmitFlag = 0;
	}

	if ((HAL_FDCAN_GetTxFifoFreeLevel(PDM.canDigital.canPeripheral) == 3) && (PDM.canDigital.transmitFlag == 1))
	{
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &PDM.canDigital.TxHeader, PDM.canDigital.TxData);
		PDM.canDigital.transmitFlag = 0;
	}

	for(uint8_t i = 0; i < 5; i++)
	{
		if ((HAL_FDCAN_GetTxFifoFreeLevel(PDM.canDriverOut[i].canPeripheral) == 3) && (PDM.canDriverOut[i].transmitFlag == 1))
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &PDM.canDriverOut[i].TxHeader, PDM.canDriverOut[i].TxData);
			PDM.canDriverOut[i].transmitFlag = 0;
		}
	}

	for(uint8_t i = 0; i < 5; i++)
	{
		if ((HAL_FDCAN_GetTxFifoFreeLevel(PDM.canDriverDiag[i].canPeripheral) == 3) && (PDM.canDriverDiag[i].transmitFlag == 1))
		{
			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &PDM.canDriverDiag[i].TxHeader, PDM.canDriverDiag[i].TxData);
			PDM.canDriverDiag[i].transmitFlag = 0;
		}
	}
}

void canFramesDefine()
{
	//Heartbeat
	PDM.canHeartBeat.canPeripheral = &hfdcan2;
	PDM.canHeartBeat.TxHeader.IdType = FDCAN_STANDARD_ID;
	PDM.canHeartBeat.TxHeader.Identifier = PDM_HEARTBEAT_ID;
	PDM.canHeartBeat.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	PDM.canHeartBeat.TxHeader.DataLength = 0x02;
	PDM.canHeartBeat.TxData[0] = 0;
	PDM.canHeartBeat.TxData[1] = 0;

	//Digital
	PDM.canDigital.canPeripheral = &hfdcan2;
	PDM.canDigital.TxHeader.IdType = FDCAN_STANDARD_ID;
	PDM.canDigital.TxHeader.Identifier = PDM_DIGITAL_ID;
	PDM.canDigital.TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	PDM.canDigital.TxHeader.DataLength = 0x02;
	PDM.canDigital.TxData[0] = 0;
	PDM.canDigital.TxData[1] = 0;

	//driver out
	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.canDriverOut[i].canPeripheral = &hfdcan2;
		PDM.canDriverOut[i].TxHeader.IdType = FDCAN_STANDARD_ID;
		PDM.canDriverOut[i].TxHeader.Identifier = 0x610 + i;
		PDM.canDriverOut[i].TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		PDM.canDriverOut[i].TxHeader.DataLength = 0x02;
		PDM.canDriverOut[i].TxData[0] = 0;
		PDM.canDriverOut[i].TxData[1] = 0;
	}


	//diagnostics
	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.canDriverDiag[i].canPeripheral = &hfdcan2;
		PDM.canDriverDiag[i].TxHeader.IdType = FDCAN_STANDARD_ID;
		PDM.canDriverDiag[i].TxHeader.Identifier = 0x620 + i;
		PDM.canDriverDiag[i].TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		PDM.canDriverDiag[i].TxHeader.DataLength = 0x06;
		PDM.canDriverDiag[i].TxData[0] = 0;
		PDM.canDriverDiag[i].TxData[1] = 0;
		PDM.canDriverDiag[i].TxData[2] = 0;
		PDM.canDriverDiag[i].TxData[3] = 0;
		PDM.canDriverDiag[i].TxData[4] = 0;
		PDM.canDriverDiag[i].TxData[5] = 0;
	}
}

void ioAssign()
{
	//SPI
	PDM.HSD_ADC1.SPI_Handle = &hspi1;
	PDM.HSD_ADC1.CS_Port = GPIOA;
	PDM.HSD_ADC1.CS_Pin = GPIO_PIN_15;

	PDM.HSD_ADC2.SPI_Handle = &hspi1;
	PDM.HSD_ADC2.CS_Port = GPIOC;
	PDM.HSD_ADC2.CS_Pin = GPIO_PIN_15;

	//LEDs
	PDM.dbgLedPort = GPIOA;
	PDM.dbgLedPin = GPIO_PIN_5;
	HAL_GPIO_WritePin(PDM.dbgLedPort, PDM.dbgLedPin, GPIO_PIN_RESET);

	//Digital Inputs
	PDM.GL_ActivePort = GPIOA;
	PDM.GL_ActivePin = GPIO_PIN_7;

	PDM.BL_ActivePort = GPIOA;
	PDM.BL_ActivePin = GPIO_PIN_6;

	PDM.Drivers[0].OutputAPort = GPIOB;
	PDM.Drivers[0].OutputAPin = GPIO_PIN_7;
	PDM.Drivers[0].OutputBPort = GPIOB;
	PDM.Drivers[0].OutputBPin = GPIO_PIN_6;
}

void initialiseADC()
{
	HAL_Delay(50);

	initADS7028(&PDM.HSD_ADC1);
	initADS7028(&PDM.HSD_ADC2);

	HAL_Delay(1000);
}
