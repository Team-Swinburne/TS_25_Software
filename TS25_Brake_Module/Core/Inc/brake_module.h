/*
 * brake_module.h
 *
 *  Created on: Aug 8, 2024
 *      Author: Lexi Boan
 */

#ifndef INC_BRAKE_MODULE_H_
#define INC_BRAKE_MODULE_H_
#include "Ticker.h"
#include "ads7028.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define DEADZONE 30
#define BRAKE_MODULE_HEARTBEAT_CAN_RATE 1000 //1000ms (1s)
#define BRAKE_MODULE_DIGITAL_CAN_RATE 50 //50ms (0.05s)
#define BRAKE_MODULE_ANALOGUE_CAN_RATE 50 //50ms (0.05s)

//put other sample(?) rates here
#define BRAKE1_VDD 5000 //5mV
#define BRAKE2_VDD 5000 //5mV
#define LOWREF_VDD 5000 //5mV
#define HIGHREF_VDD 5000 //5mV
#define DIGITAL_VDD 5000 //5mV

//External variables from main.c
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef hspi1;

//Function prototypes
void TransmitHeartBeat();
void TransmitDigital();
void UpdateDigital();
void TransmitAnalogue();
void UpdateAnalogue();
void canFramesDefine();
void ioAssign();
void initialiseADC();

//put can struct here?
typedef struct canFrame {
	FDCAN_HandleTypeDef canPeripheral;
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t              TxData[8];
	//uint32_t             TxMailbox;
	uint8_t				 transmitFlag; //1 = ready to transmit
} canFrame_t;


typedef struct BrakeModuleInfo
{
	uint8_t HeartBeatState; //0 = fine
	uint8_t HeartBeatCounter;

	float Brake1_Raw;
	float Brake2_Raw;
	float LowRef;
	float HighRef1;

	uint16_t Brake1_RawInt;
	uint16_t Brake2_RawInt;
	uint16_t LowRefInt;
	uint16_t HighRef1Int;

	// Raw voltage value, ready to be converted to 8 bits
	uint16_t LowPressureRaw;
	uint16_t HighPressureRaw;
	uint16_t BSPDLatchRaw;
	uint16_t GL_InOKRaw;

	uint8_t LowPressure;
	uint8_t HighPressure;
	uint8_t BSPDLatch;
	uint8_t BSPD_OK;
	uint8_t FiveKW;
	uint8_t GL_InOK;

	//put can variables here
    canFrame_t canHeartBeat;
    canFrame_t canAnalogue;
    canFrame_t canDigital;
    canFrame_t canDigitalRaw;

	//Debug LED
	GPIO_TypeDef* dbgLedPort;
	uint16_t dbgLedPin;

	//Digital Inputs
	GPIO_TypeDef* BSPD_OK_InPort;
	uint16_t BSPD_OK_InPin;
	GPIO_TypeDef* FiveKWInPort;
	uint16_t FiveKWInPin;

	//ADC Analogue Inputs
	ADS7028_HandleTypeDef Brake_ADC1;

} BrakeModuleInfo_t;

extern BrakeModuleInfo_t BrakeModule;

#endif /* INC_BRAKE_MODULE_H_ */
