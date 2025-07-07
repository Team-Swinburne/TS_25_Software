/*
 * Discharge.c
 *
 *  Created on: Jul 27, 2024
 *      Author: Ethan Jones
 */

#include <UCM.h>

UCMInfo_t UCM;
uint16_t ThermistorResistance;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

void CheckReceivedCAN()
{
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if ((RxHeader.StdId == (UCM_EEPROM_RECEIVE_1_ID + UCM.Number)) && (RxHeader.DLC == 5))
	{
		  EEPROM_24FC_ByteWrite(0, RxData[0]);

		  //3.3V calibration
		  EEPROM_24FC_ByteWrite(1, RxData[2]);
		  EEPROM_24FC_ByteWrite(2, RxData[1]);

		  //5.0V calibration
		  EEPROM_24FC_ByteWrite(3, RxData[4]);
		  EEPROM_24FC_ByteWrite(4, RxData[3]);

		  ReadEEPROMconfig();
		  ConfigurePullUpDownResistors();
		  ConfigureDigitalOutputs();
		  canFramesDefine();
		  ConfigureReceiveFilter();

		  RxHeader.StdId = 0;
	}

	if ((RxHeader.StdId == (UCM_EEPROM_RECEIVE_2_ID + UCM.Number)) && (RxHeader.DLC == 8))
	{
		  //IO bias
		  for(int i = 0; i < 4; i++)
		  {
			  EEPROM_24FC_ByteWrite(0x05 + i, RxData[i]);
		  }

		  //IO config
		  for(int i = 0; i < 4; i++)
		  {
			  EEPROM_24FC_ByteWrite(0x09 + i, RxData[4 + i]);
		  }

		  ReadEEPROMconfig();
		  ConfigurePullUpDownResistors();
		  ConfigureDigitalOutputs();
		  canFramesDefine();
		  ConfigureReceiveFilter();

		  RxHeader.StdId = 0;
	}

	if ((RxHeader.StdId == (UCM_EEPROM_RECEIVE_3_ID + UCM.Number)) && (RxHeader.DLC == 4))
	{
		  for(int i = 0; i < 4; i++)
		  {
			  EEPROM_24FC_ByteWrite(i + 13, RxData[i]);
		  }

		  ReadEEPROMconfig();
		  ConfigurePullUpDownResistors();
		  ConfigureDigitalOutputs();
		  canFramesDefine();
		  ConfigureReceiveFilter();

		  RxHeader.StdId = 0;
	}
}

void ReadEEPROMconfig()
{
	uint8_t EEPROM_data[17];

	for(int i = 0; i < 16; i++)
	{
		EEPROM_data[i] = EEPROM_24FC_ByteRead(i);
	}

	UCM.Number = EEPROM_data[0];

	UCM.Voltage3V3 = (256*EEPROM_data[2] + EEPROM_data[1]);

	UCM.Voltage5V = (256*EEPROM_data[4] + EEPROM_data[3]);

	for(uint8_t i = 0; i < 4; i++)
	{
		UCM.IObias[i] = EEPROM_data[5 + i];
		UCM.IOconfig[i] = EEPROM_data[9 + i];
	}

	UCM.DiffVoltageEnabled[0] = EEPROM_data[13];
	UCM.DiffVoltageEnabled[1] = EEPROM_data[14];

	UCM.DiffVoltageMux[0] = EEPROM_data[15];
	UCM.DiffVoltageMux[1] = EEPROM_data[16];
}

void ConfigureReceiveFilter()
{
	  CAN_FilterTypeDef canfilterconfig;

	  uint32_t filter_mask = 0x1FFFFF0F;
	  uint32_t filter_id = 0x440 + UCM.Number;

	  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)
	  canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones
	  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  canfilterconfig.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF;//0x446<<5;
	  canfilterconfig.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8;//0;
	  canfilterconfig.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;//0x446<<5;
	  canfilterconfig.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;//0x0000;

	  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

void ConfigurePullUpDownResistors()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	for(uint8_t i = 0; i < 4; i++)
	{
		GPIO_InitStruct.Pin = UCM.IOBiasPin[i];

		if(UCM.IObias[i] == 1) //Pull-up
		{
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(UCM.IOBiasPort[i], &GPIO_InitStruct);

			HAL_GPIO_WritePin(UCM.IOBiasPort[i], UCM.IOBiasPin[i], GPIO_PIN_SET);
		}
		else if(UCM.IObias[i] == 2) //Pull-down
		{
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(UCM.IOBiasPort[i], &GPIO_InitStruct);

			HAL_GPIO_WritePin(UCM.IOBiasPort[i], UCM.IOBiasPin[i], GPIO_PIN_RESET);
		}
		else //Floating
		{
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(UCM.IOBiasPort[i], &GPIO_InitStruct);
		}
	}
}

void ConfigureDigitalOutputs()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint16_t TimerChannelMapping[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
	uint16_t OutputMicroPinMapping[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_2};

	for(uint8_t i = 0; i < 4; i++)
	{
		if(UCM.IOconfig[i] == 0x04)
		{
			GPIO_InitStruct.Pin = OutputMicroPinMapping[i];
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

			HAL_TIM_PWM_Init(&htim2);

			TIM_OC_InitTypeDef sConfigOC = {0};
			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = 0;
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TimerChannelMapping[i]);

			if(htim2.State == HAL_TIM_STATE_READY)
			{
				HAL_TIM_Base_Start_IT(&htim2);
			}

			HAL_TIM_PWM_Start(&htim2, TimerChannelMapping[i]);
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim2, TimerChannelMapping[i]);

			GPIO_InitStruct.Pin = OutputMicroPinMapping[i];
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}
	}
}

void ReadSingleEndedVoltage()
{
	for(uint8_t i = 0; i < 4; i++)
	{
		if(UCM.IOconfig[i] == 0x05)
		{
			//Mapping from i to MUX selection
			uint8_t MuxSelection[4] = {ADS122C04_MUX_AIN3_AVSS,
									   ADS122C04_MUX_AIN2_AVSS,
									   ADS122C04_MUX_AIN1_AVSS,
									   ADS122C04_MUX_AIN0_AVSS};

			ADS122C04Reg_t ADS122C04_Config;
			ADS122C04_Config = ADS122C04_DefaultConfig;

			ADS122C04_Config.reg0.bit.GAIN = ADS122C04_GAIN_1;
			ADS122C04_Config.reg1.bit.VREF = ADS122C04_VREF_EXT_REF_PINS;
			ADS122C04_Config.reg1.bit.MODE = ADS122C04_OP_MODE_TURBO;
			ADS122C04_Config.reg1.bit.DR = ADS122C04_DATA_RATE_1000SPS;
			ADS122C04_Config.reg0.bit.MUX = MuxSelection[i];

			ADS122C04_Configurate(&ADS122C04_Config);

			ADS122C04_WriteStartSync();
			HAL_Delay(100);
			uint32_t ADCcount = ADS122C04_ReadData();
			UCM.SingleEndedVoltage[i] = UCM.Voltage5V*(ADCcount/8388608.0);
		}
	}
}

void ReadDiffVoltage()
{
	for(uint8_t i = 0; i < 2; i++)
	{
		if(UCM.DiffVoltageEnabled[i] == 0x01)
		{
			ADS122C04Reg_t ADS122C04_Config;
			ADS122C04_Config = ADS122C04_DefaultConfig;

			ADS122C04_Config.reg0.bit.GAIN = ADS122C04_GAIN_128;
			ADS122C04_Config.reg1.bit.VREF = ADS122C04_VREF_EXT_REF_PINS;
			ADS122C04_Config.reg1.bit.MODE = ADS122C04_OP_MODE_TURBO;
			ADS122C04_Config.reg1.bit.DR = ADS122C04_DATA_RATE_20SPS;

			ADS122C04_Config.reg0.bit.MUX = UCM.DiffVoltageMux[i];

			ADS122C04_Configurate(&ADS122C04_Config);

			ADS122C04_WriteStartSync();
			HAL_Delay(100);
			uint32_t ADCcount = ADS122C04_ReadData();

			if(ADCcount > 8388608)
			{
				ADCcount = 16777216 - ADCcount;
				UCM.DiffEndedVoltage[i] = -(UCM.Voltage5V*1000.0)*((ADCcount)/8388608.0)*(1/128.0);
			}
			else
			{
				UCM.DiffEndedVoltage[i] = (UCM.Voltage5V*1000.0)*((ADCcount)/8388608.0)*(1/128.0);
			}
		}
	}
}

void TransmitHeartBeat()
{
	//Increment counter by 1, if 255 force to 0
	if(UCM.HeartBeatCounter == 255)
	{
		UCM.HeartBeatCounter = 0;
	}
	else
	{
		UCM.HeartBeatCounter++;
	}
	UCM.canHeartBeat.TxData[1] = UCM.HeartBeatCounter;

	//Toggle debug LED
	HAL_GPIO_TogglePin(UCM.dbgLedPort, UCM.dbgLedPin);

	HAL_CAN_AddTxMessage(&hcan, &UCM.canHeartBeat.TxHeader, UCM.canHeartBeat.TxData, &UCM.canHeartBeat.TxMailbox);
}

void TransmitDigital()
{
	UCM.canDigital.TxData[0] = UCM.digitalIn1Read;
	UCM.canDigital.TxData[1] = UCM.digitalIn2Read;
	UCM.canDigital.TxData[2] = UCM.digitalIn3Read;
	UCM.canDigital.TxData[3] = UCM.digitalIn4Read;

	HAL_CAN_AddTxMessage(&hcan, &UCM.canDigital.TxHeader, UCM.canDigital.TxData, &UCM.canDigital.TxMailbox);
}

void UpdateDigital()
{
	UCM.digitalIn1Read = HAL_GPIO_ReadPin(UCM.digitalIn1Port, UCM.digitalIn1Pin);
	UCM.digitalIn2Read = HAL_GPIO_ReadPin(UCM.digitalIn2Port, UCM.digitalIn2Pin);
	UCM.digitalIn3Read = HAL_GPIO_ReadPin(UCM.digitalIn3Port, UCM.digitalIn3Pin);
	UCM.digitalIn4Read = HAL_GPIO_ReadPin(UCM.digitalIn4Port, UCM.digitalIn4Pin);
}

void OffsetCalibrationADC()
{
	  ADS122C04Reg_t ADS122C04_Config;
	  ADS122C04_Config = ADS122C04_DefaultConfig;
	  ADS122C04_Config.reg0.bit.MUX = ADS122C04_MUX_SHORTED;//ADS122C04_MUX_AIN3_AIN2;
	  ADS122C04_Config.reg0.bit.GAIN = ADS122C04_GAIN_128;
	  ADS122C04_Config.reg1.bit.VREF = ADS122C04_VREF_EXT_REF_PINS;
	  ADS122C04_Config.reg1.bit.MODE = ADS122C04_OP_MODE_TURBO;
	  ADS122C04_Config.reg1.bit.DR = ADS122C04_DATA_RATE_20SPS;

	  ADS122C04_Configurate(&ADS122C04_Config);

	  ADS122C04_WriteStartSync();
	  HAL_Delay(100);
	  UCM.OffsetCalibration = ADS122C04_ReadData();
}

void UpdateOutputBiasResistors()
{
	if(UCM.IO1Bias == 0) return;

	//HAL_GPIO_WritePin(UCM.IO1BiasPort, UCM.IO1BiasPin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(UCM.IO2BiasPort, UCM.IO2BiasPin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(UCM.IO3BiasPort, UCM.IO3BiasPin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(UCM.IO4BiasPort, UCM.IO4BiasPin, GPIO_PIN_SET);
}

void TransmitIOBiasStatus()
{
	UCM.canBiasState.TxData[0] = UCM.IO1Bias;
	UCM.canBiasState.TxData[1] = UCM.IO2Bias;
	UCM.canBiasState.TxData[2] = UCM.IO3Bias;
	UCM.canBiasState.TxData[3] = UCM.IO4Bias;

	HAL_CAN_AddTxMessage(&hcan, &UCM.canBiasState.TxHeader, UCM.canBiasState.TxData, &UCM.canBiasState.TxMailbox);
}

void TransmitSingleEndedVoltage()
{
	UCM.canSingleEndedVoltage.TxData[0] = (UCM.SingleEndedVoltage[0] >> 8);
	UCM.canSingleEndedVoltage.TxData[1] = (UCM.SingleEndedVoltage[0] & 0xFF);
	UCM.canSingleEndedVoltage.TxData[2] = (UCM.SingleEndedVoltage[1] >> 8);
	UCM.canSingleEndedVoltage.TxData[3] = (UCM.SingleEndedVoltage[1] & 0xFF);
	UCM.canSingleEndedVoltage.TxData[4] = (UCM.SingleEndedVoltage[2] >> 8);
	UCM.canSingleEndedVoltage.TxData[5] = (UCM.SingleEndedVoltage[2] & 0xFF);
	UCM.canSingleEndedVoltage.TxData[6] = (UCM.SingleEndedVoltage[3] >> 8);
	UCM.canSingleEndedVoltage.TxData[7] = (UCM.SingleEndedVoltage[3] & 0xFF);

	HAL_CAN_AddTxMessage(&hcan, &UCM.canSingleEndedVoltage.TxHeader, UCM.canSingleEndedVoltage.TxData, &UCM.canSingleEndedVoltage.TxMailbox);
}

void TransmitDifferentialVoltage()
{
	UCM.canDifferentialVoltage.TxData[0] = (UCM.DiffEndedVoltage[0] >> 8);
	UCM.canDifferentialVoltage.TxData[1] = (UCM.DiffEndedVoltage[0] & 0xFF);
	UCM.canDifferentialVoltage.TxData[2] = (UCM.DiffEndedVoltage[1] >> 8);
	UCM.canDifferentialVoltage.TxData[3] = (UCM.DiffEndedVoltage[1] & 0xFF);

	HAL_CAN_AddTxMessage(&hcan, &UCM.canDifferentialVoltage.TxHeader, UCM.canDifferentialVoltage.TxData, &UCM.canDifferentialVoltage.TxMailbox);
}

/*
void CheckReceivedCAN()
{
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	if ((RxHeader.StdId == 0x446))
	  {
		  for(int i = 0; i < 8; i++)
		  {
			  EEPROM_24FC_ByteWrite(i, 0xFF);
		  }
	  }
}
*/

void canFramesDefine()
{
	//Heartbeat
	UCM.canHeartBeat.canPeripheral = hcan;
	UCM.canHeartBeat.TxHeader.IDE = CAN_ID_STD;
	UCM.canHeartBeat.TxHeader.StdId = (0x400 + UCM.Number);
	UCM.canHeartBeat.TxHeader.RTR = CAN_RTR_DATA;
	UCM.canHeartBeat.TxHeader.DLC = 0x02;
	UCM.canHeartBeat.TxData[0] = 0;
	UCM.canHeartBeat.TxData[1] = 0;

	//Single-ended voltage
	UCM.canSingleEndedVoltage.canPeripheral = hcan;
	UCM.canSingleEndedVoltage.TxHeader.IDE = CAN_ID_STD;
	UCM.canSingleEndedVoltage.TxHeader.StdId = (0x410 + UCM.Number);
	UCM.canSingleEndedVoltage.TxHeader.RTR = CAN_RTR_DATA;
	UCM.canSingleEndedVoltage.TxHeader.DLC = 0x08;
	UCM.canSingleEndedVoltage.TxData[0] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[1] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[2] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[3] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[4] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[5] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[6] = 0xAA;
	UCM.canSingleEndedVoltage.TxData[7] = 0xAA;

	//Differential voltage
	UCM.canDifferentialVoltage.canPeripheral = hcan;
	UCM.canDifferentialVoltage.TxHeader.IDE = CAN_ID_STD;
	UCM.canDifferentialVoltage.TxHeader.StdId = (0x420 + UCM.Number);
	UCM.canDifferentialVoltage.TxHeader.RTR = CAN_RTR_DATA;
	UCM.canDifferentialVoltage.TxHeader.DLC = 0x04;
	UCM.canDifferentialVoltage.TxData[0] = 0xAA;
	UCM.canDifferentialVoltage.TxData[1] = 0xAA;
	UCM.canDifferentialVoltage.TxData[2] = 0xAA;
	UCM.canDifferentialVoltage.TxData[3] = 0xAA;
}

void SingleEndedRead()
{

}

void ioAssign()
{
	//Outputs
	UCM.dbgLedPort = GPIOB;
	UCM.dbgLedPin = GPIO_PIN_1;
	HAL_GPIO_WritePin(UCM.dbgLedPort, UCM.dbgLedPin, GPIO_PIN_RESET); //LED on (default)

	//Digital inputs
    UCM.digitalIn1Port = GPIOB;
    UCM.digitalIn1Pin = GPIO_PIN_3;
    UCM.digitalIn2Port = GPIOB;
    UCM.digitalIn2Pin = GPIO_PIN_4;
    UCM.digitalIn3Port = GPIOB;
    UCM.digitalIn3Pin = GPIO_PIN_5;
    UCM.digitalIn4Port = GPIOA;
    UCM.digitalIn4Pin = GPIO_PIN_15;

	//IO bias pins
    UCM.IOBiasPort[0] = GPIOA;
    UCM.IOBiasPin[0] = GPIO_PIN_5;
    UCM.IOBiasPort[1] = GPIOA;
    UCM.IOBiasPin[1] = GPIO_PIN_6;
    UCM.IOBiasPort[2] = GPIOA;
    UCM.IOBiasPin[2] = GPIO_PIN_7;
    UCM.IOBiasPort[3] = GPIOA;
    UCM.IOBiasPin[3] = GPIO_PIN_4;
}
