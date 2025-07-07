/*
 * Discharge.h
 *
 *  Created on: Jul 22, 2024
 *      Author: Ethan Jones
 */

#ifndef INC_UCM_H_
#define INC_UCM_H_
#include "Ticker.h"
#include "EEPROM_24FC.h"
#include "ads122c04.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define UCM_EEPROM_RECEIVE_1_ID 0x430
#define UCM_EEPROM_RECEIVE_2_ID 0x440
#define UCM_EEPROM_RECEIVE_3_ID 0x450

#define CAN_RATE_1000MS 1000 //1000ms
#define CAN_RATE_50MS 50 // 50ms

#define IO_CHANNEL_1 0
#define IO_CHANNEL_2 1
#define IO_CHANNEL_3 2
#define IO_CHANNEL_4 3

//External variables (declared in main.c)
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;

//Function prototypes
void ConfigureReceiveFilter();
void ReadEEPROMconfig();
void ConfigurePullUpDownResistors();
void ReadSingleEndedVoltage();
void ReadDiffVoltage();
void ConfigureDigitalOutputs();

void TransmitHeartBeat();
void TransmitDigital();
void UpdateDigital();
void TransmitAnalogue();
void TransmitSingleEndedVoltage();
void TransmitDifferentialVoltage();
void UpdateAnalogue();
void CheckReceivedCAN();
void canFramesDefine();
void ioAssign();
void UpdateOutputBiasResistors();
void TransmitIOBiasStatus();

typedef struct canFrame {
	CAN_HandleTypeDef canPeripheral;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t              TxData[8];
	uint32_t             TxMailbox;
} canFrame_t;

typedef struct UCMInfo {
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

    uint16_t SingleEndedVoltage[4]; //1mV resolution
    int16_t DiffEndedVoltage[2]; //1uV resolution, signed

    uint8_t DiffVoltageEnabled[2];
    uint8_t DiffVoltageMux[2];

    uint8_t DischargeDisable;
    uint8_t FiveKW;
    uint8_t PDOC_ok;

    canFrame_t canHeartBeat;
    canFrame_t canDigital;
    canFrame_t canBiasState;
    canFrame_t canEEPROMtest;
    canFrame_t canSingleEndedVoltage;
    canFrame_t canDifferentialVoltage;

    //Config from EEPROM
    uint8_t Number;
    uint16_t Voltage3V3;
    uint16_t Voltage5V;
    uint8_t IObias[4];
    uint8_t IOconfig[4];

    uint8_t Test[8];

    uint32_t OffsetCalibration;

    //Debug LED
    GPIO_TypeDef* dbgLedPort;
    uint16_t dbgLedPin;

    //Digital Input states
    uint8_t digitalIn1Read;
    uint8_t digitalIn2Read;
    uint8_t digitalIn3Read;
    uint8_t digitalIn4Read;

    //IO bias (Floating = 0, Pull-down = 1, Pull-up = 2)
    uint8_t IO1Bias;
    uint8_t IO2Bias;
    uint8_t IO3Bias;
    uint8_t IO4Bias;

    //Digital inputs
    GPIO_TypeDef* digitalIn1Port;
    uint16_t digitalIn1Pin;
    GPIO_TypeDef* digitalIn2Port;
    uint16_t digitalIn2Pin;
    GPIO_TypeDef* digitalIn3Port;
    uint16_t digitalIn3Pin;
    GPIO_TypeDef* digitalIn4Port;
    uint16_t digitalIn4Pin;

    //IO bias pins
    GPIO_TypeDef* IOBiasPort[4];
    uint16_t IOBiasPin[4];
    //GPIO_TypeDef* IO2BiasPort;
    //uint16_t IO2BiasPin;
    //GPIO_TypeDef* IO3BiasPort;
    //uint16_t IO3BiasPin;
    //GPIO_TypeDef* IO4BiasPort;
    //uint16_t IO4BiasPin;

} UCMInfo_t;

extern UCMInfo_t UCM;

#endif /* INC_UCM_H_ */
