/*
 * Discharge.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Thomas Orr
 */

#ifndef INC_DISCHARGE_H_
#define INC_DISCHARGE_H_
#include "Ticker.h"
#include "ads7028.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define DISCHARGE_HEARTBEAT_CAN_RATE 1000 // 1000ms
#define DISCHARGE_ANALOGUE_CAN_RATE 50 // 50ms
#define DISCHARGE_DIGITAL_CAN_RATE 50 // 50ms

// Will need to be replaced with new sample
#define HV_R1 1314000
#define HV_R2 6660
#define THERM_BIAS_RESIS 1982
#define PDOC_VDD 3394.0 //3.394 mV
#define HV_SENSE_VDD 3351.0 //3.351 mV
#define THERMISTOR_BETA 3350 //Thermistor beta


// External variables (declared in main.c)
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef hspi1;

//Function prototypes
void TransmitHeartBeat();
void TransmitDigital();
void UpdateDigital();
void TransmitAnalogue();
void TransmitAnalogueRaw();
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

typedef struct DischargeInfo {
	uint8_t HeartBeatState; //0 = fine
    uint8_t HeartBeatCounter;

    uint16_t PDOC_Ref_Raw_Voltage; // 10mV resolution
    uint16_t PDOC_Ref_Temp; // 0.1C resolution
    uint16_t HV_Sense_Raw_Voltage; // 10mV resolution
    uint16_t MC_Voltage; // 0.1V resolution
    uint16_t HV_Sense_Ref_Raw_Voltage; // 10mV resolution
    uint16_t HV_Active_Voltage; // 0.1V resolution
    uint16_t PDOC_Sensor_Raw_Voltage; // 10mV resolution
    uint16_t PDOC_Sensor_Temp; // 0.1C resolution

    uint8_t DischargeDisable;
    uint8_t FiveKW;
    uint8_t PDOC_ok;

    canFrame_t canHeartBeat;
    canFrame_t canAnalogue;
    canFrame_t canAnalogueRaw;
    canFrame_t canDigital;

    //Debug LED
    GPIO_TypeDef* dbgLedPort;
    uint16_t dbgLedPin;

    //Digital inputs
    GPIO_TypeDef* DischargeDisableInPort;
    uint16_t DischargeDisableInPin;
    GPIO_TypeDef* FiveKWInPort;
    uint16_t FiveKWInPin;
    GPIO_TypeDef* PDOCokInPort;
    uint16_t PDOCokInPin;

    //SPI ADC analogue inputs
    ADS7028_HandleTypeDef HVSense_ADC;
    ADS7028_HandleTypeDef PDOC_ADC;
} DischargeInfo_t;

extern DischargeInfo_t Discharge;

#endif /* INC_DISCHARGE_H_ */
