/*
 * TempModule.h
 *
 *  Created on: Jul 4, 2025
 *      Author: Lexi Boan
 *
 *  V1 - Heartbeat Code: Jul 4, 2025
 *  V2 - CAN Configuration: Jul 7, 2025
 */

#ifndef INC_TEMPMODULE_H_
#define INC_TEMPMODULE_H_
#include "Ticker.h"
#include "ads7028.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define CAN_RATE_1000MS 1000 //1000ms

// Temp bank voltage samples
#define TB1_VDD 5131.0
#define TB2_VDD 5101.0
#define TB3_VDD 5019.0
#define TB4_VDD 5078.0
#define TB5_VDD 4979.0

#define NUM_OF_TEMPBANKS 5

// External variables (declared in main.c)
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef hspi1;

// Function prototypes
void CalculateVoltageReading();
void CalculateTemperature();
void TransmitHeartBeat();
void TransmitAnalogue();
void TransmitAnalogueRaw();
void TransmitCAN();
void UpdateAnalogue();
void canFramesDefine();
void ioAssign();
void initialiseADC();

typedef struct canFrame {
	FDCAN_HandleTypeDef* canPeripheral;
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t              TxData[8];
	//uint32_t             TxMailbox;
	uint8_t 			transmitFlag; //1 = Ready to transmit
} canFrame_t;

typedef struct tempBank {
	int8_t Temperature[8];
	uint16_t RawVoltages[8];
    uint32_t RawVoltagesSum[8];
    uint16_t RawVoltageCount[8];
} tempBank_t;

typedef struct TempModuleInfo
{
	uint8_t HeartBeatState; //0 = fine, 1 = error
    uint8_t HeartBeatCounter;

    tempBank_t TempBanks[6];

    uint8_t AverageFlag; //averages out the temp bank voltages when = 1

    canFrame_t canHeartBeat;
    //canFrame_t canTempModuleOrion; //Message for Orion BMS
    canFrame_t canAnalogue[6];
    canFrame_t canAnalogueRaw[12];

    //Debug LED
    GPIO_TypeDef* dbgLedPort;
    uint16_t dbgLedPin;

    //SPI ADC analogue inputs
    ADS7028_HandleTypeDef ADC_Bank[6];
} TempModuleInfo_t;

extern TempModuleInfo_t TempModule;


#endif /* INC_TEMPMODULE_H_ */
