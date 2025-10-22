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

			PDM.driverSwitch[i].Grab[HSD_A].DutyCycle = RxData[0];
			PDM.driverSwitch[i].Grab[HSD_B].DutyCycle = RxData[1];
		}
	}

}

void updatePWM()
{
	//Pump 1
	TIM2->CCR1 = 31999*(PDM.driverSwitch[0].Grab[HSD_A].DutyCycle/100.0);

	//Fan 1
	TIM2->CCR2 = 31999*(PDM.driverSwitch[0].Grab[HSD_B].DutyCycle/100.0);

	//Pump 2
	TIM2->CCR3 = 31999*(PDM.driverSwitch[1].Grab[HSD_A].DutyCycle/100.0);

	//Fan 2
	TIM2->CCR4 = 31999*(PDM.driverSwitch[1].Grab[HSD_B].DutyCycle/100.0);

	//Fan 3
	TIM4->CCR4 = 31999*(PDM.driverSwitch[2].Grab[HSD_B].DutyCycle/100.0);

	//Accumulator
	PDM.diagAccum = PDM.driverSwitch[2].Grab[HSD_A].DutyCycle;

	//NMEA
	PDM.diagNMEA = PDM.driverSwitch[3].Grab[HSD_A].DutyCycle;

	//Discharge
	PDM.diagDschrg = PDM.driverSwitch[3].Grab[HSD_B].DutyCycle;

	//Motor Controller (Inverter)
	PDM.diagInverter = PDM.driverSwitch[4].Grab[HSD_A].DutyCycle;

	//Brake Light
	PDM.diagBrkLght = PDM.driverSwitch[4].Grab[HSD_B].DutyCycle;
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
	HAL_GPIO_WritePin(PDM.Drivers[3].OutputBPort, PDM.Drivers[3].OutputBPin, 1); //Discharge
	HAL_GPIO_WritePin(PDM.Drivers[4].OutputBPort, PDM.Drivers[4].OutputBPin, PDM.diagBrkLght); //Brake Light

	if(PDM.BL_Active)
	{
		HAL_GPIO_WritePin(PDM.Drivers[2].OutputBPort, PDM.Drivers[2].OutputBPin, 1); //Accumulator
		HAL_GPIO_WritePin(PDM.Drivers[3].OutputAPort, PDM.Drivers[3].OutputAPin, 1); //NMEA

		if(PDM.GL_Active)
		{
			HAL_GPIO_WritePin(PDM.Drivers[4].OutputAPort, PDM.Drivers[4].OutputAPin, 1); //Inverter
			HAL_GPIO_WritePin(PDM.Drivers[2].OutputAPort, PDM.Drivers[2].OutputAPin, TIM4->CCR4); //Fan 3
			HAL_GPIO_WritePin(PDM.Drivers[1].OutputBPort, PDM.Drivers[1].OutputBPin, TIM2->CCR4); //Fan 2
			HAL_GPIO_WritePin(PDM.Drivers[0].OutputBPort, PDM.Drivers[0].OutputBPin, TIM2->CCR2); //Fan 1
			HAL_GPIO_WritePin(PDM.Drivers[1].OutputAPort, PDM.Drivers[1].OutputAPin, TIM2->CCR3); //Pump 2
			HAL_GPIO_WritePin(PDM.Drivers[0].OutputAPort, PDM.Drivers[0].OutputAPin, TIM2->CCR1); //Pump 1
		}
		else
		{
			HAL_GPIO_WritePin(PDM.Drivers[4].OutputAPort, PDM.Drivers[4].OutputAPin, PDM.diagInverter); //Inverter
			HAL_GPIO_WritePin(PDM.Drivers[2].OutputAPort, PDM.Drivers[2].OutputAPin, TIM4->CCR4); //Fan 3
			HAL_GPIO_WritePin(PDM.Drivers[1].OutputBPort, PDM.Drivers[1].OutputBPin, TIM2->CCR4); //Fan 2
			HAL_GPIO_WritePin(PDM.Drivers[0].OutputBPort, PDM.Drivers[0].OutputBPin, TIM2->CCR2); //Fan 1
			HAL_GPIO_WritePin(PDM.Drivers[1].OutputAPort, PDM.Drivers[1].OutputAPin, TIM2->CCR3); //Pump 2
			HAL_GPIO_WritePin(PDM.Drivers[0].OutputAPort, PDM.Drivers[0].OutputAPin, TIM2->CCR1); //Pump 1
		}

	}
	else
	{
		HAL_GPIO_WritePin(PDM.Drivers[2].OutputBPort, PDM.Drivers[2].OutputBPin, PDM.diagAccum); //Accumulator
		HAL_GPIO_WritePin(PDM.Drivers[3].OutputAPort, PDM.Drivers[3].OutputAPin, PDM.diagNMEA); //NMEA
	}

	for(uint8_t i = 0; i < 5; i++)
	{
		PDM.Drivers[i].OutputA = HAL_GPIO_ReadPin(PDM.Drivers[i].OutputAPort, PDM.Drivers[i].OutputAPin);
		PDM.Drivers[i].OutputB = HAL_GPIO_ReadPin(PDM.Drivers[i].OutputBPort, PDM.Drivers[i].OutputBPin);
	}
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
	PDM.GL_ActivePin = GPIO_PIN_6;

	PDM.BL_ActivePort = GPIOA;
	PDM.BL_ActivePin = GPIO_PIN_7;

	PDM.Drivers[0].OutputAPort = GPIOB;
	PDM.Drivers[0].OutputAPin = GPIO_PIN_7; //pump 1
	PDM.Drivers[0].OutputBPort = GPIOA;
	PDM.Drivers[0].OutputBPin = GPIO_PIN_11; //fan 1

	PDM.Drivers[1].OutputAPort = GPIOB;
	PDM.Drivers[1].OutputAPin = GPIO_PIN_8; //pump 2
	PDM.Drivers[1].OutputBPort = GPIOC;
	PDM.Drivers[1].OutputBPin = GPIO_PIN_14; //fan 2

	PDM.Drivers[2].OutputAPort = GPIOA;
	PDM.Drivers[2].OutputAPin = GPIO_PIN_9; //spare (inverter fan)
	PDM.Drivers[2].OutputBPort = GPIOA;
	PDM.Drivers[2].OutputBPin = GPIO_PIN_10; //accumulator

	PDM.Drivers[3].OutputAPort = GPIOC;
	PDM.Drivers[3].OutputAPin = GPIO_PIN_6; //NMEA
	PDM.Drivers[3].OutputBPort = GPIOA;
	PDM.Drivers[3].OutputBPin = GPIO_PIN_12; //Discharge

	PDM.Drivers[4].OutputAPort = GPIOA;
	PDM.Drivers[4].OutputAPin = GPIO_PIN_8; //Inverter
	PDM.Drivers[4].OutputBPort = GPIOB;
	PDM.Drivers[4].OutputBPin = GPIO_PIN_6; //Brake Light
}

void initialiseADC()
{
	HAL_Delay(50);

	initADS7028(&PDM.HSD_ADC1);
	initADS7028(&PDM.HSD_ADC2);

	HAL_Delay(1000);
}
