/*
 * PDM.h
 *
 *  Created on: Aug 14, 2025
 *      Author: Lexi Boan
 */

#ifndef INC_PDM_H_
#define INC_PDM_H_
#include "Ticker.h"
#include "ads7028.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/*
#define PDM_RECEIVE_DRIVER1_ID 0x260;
#define PDM_RECEIVE_DRIVER2_ID 0x261;
#define PDM_RECEIVE_DRIVER3_ID 0x262;
#define PDM_RECEIVE_DRIVER4_ID 0x263;
#define PDM_RECEIVE_DRIVER5_ID 0x264;
*/

#define PDM_HEARTBEAT_ID 0x600
#define PDM_HEARTBEAT_CAN_RATE 1000 //1000ms

#define PDM_DIGITAL_ID 0x601
#define PDM_DIGITAL_CAN_RATE 50 //50ms

/*
#define PDM_DRIVER1_OUT_ID 0x610
#define PDM_DRIVER2_OUT_ID 0x611
#define PDM_DRIVER3_OUT_ID 0x612
#define PDM_DRIVER4_OUT_ID 0x613
#define PDM_DRIVER5_OUT_ID 0x614
*/

/*
#define PDM_DRIVER1_DIAG_ID 0x620
#define PDM_DRIVER2_DIAG_ID 0x621
#define PDM_DRIVER3_DIAG_ID 0x622
#define PDM_DRIVER4_DIAG_ID 0x623
#define PDM_DRIVER5_DIAG_ID 0x624
*/

#define PDM_HSDRIVERS_CAN_RATE 50 //50ms

//sample rates for HSDs
#define HSD_VDD 5000 	//5V

//External variables (declared in main.c)
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef hspi1;

//Function prototypes
void CheckReceivedCAN();
void updatePWM();
void TransmitHeartBeat();
void TransmitDigital();
void UpdateDigital();
void TransmitDriverOut();
void UpdateDriverOut();
void TransmitHSDrivers();
void UpdateHSDrivers();
void TransmitCAN();
void canFramesDefine();
void ioAssign();
void initialiseADC();

#define HSD_A 0
#define HSD_B 1

typedef struct canFrame
{
	FDCAN_HandleTypeDef* canPeripheral;
	FDCAN_TxHeaderTypeDef TxHeader;
	uint8_t					TxData[8];
	uint8_t 			transmitFlag; //1 = Ready to transmit
} canFrame_t;

typedef struct HighSideDriverReceive {
    uint16_t Current;	//current, 0.001A resolution
    uint8_t DutyCycle;
} HighSideReceive_t;

typedef struct HighSideDriverConf {
    uint16_t Current;	//current, 0.001A resolution
    uint8_t DutyCycle;
} HighSideDriver_t;

typedef struct hsdToggle
{
	HighSideReceive_t Grab[2];
    HighSideDriver_t Driver[2];
} hsdToggle_t;

typedef struct hsdOutput {
	uint8_t OutputA; 	//digital out
    uint8_t OutputB; 	//digital out
    GPIO_TypeDef* OutputAPort;
    uint16_t OutputAPin;
    GPIO_TypeDef* OutputBPort;
    uint16_t OutputBPin;
} hsdOut_t;

typedef struct PDMInfo {

	uint8_t HeartBeatState; //0 = fine
    uint8_t HeartBeatCounter;

    uint8_t GL_Active;
    uint8_t BL_Active;

    hsdOut_t Drivers[5];
    hsdToggle_t driverSwitch[5];

    canFrame_t canHeartBeat;
    canFrame_t canDigital;
    canFrame_t canDriverOut[5];
    canFrame_t canDriverDiag[5];

    float diagBrkLght;
    float diagInverter;
    float diagDschrg;
    float diagNMEA;
    float diagAccum;

    //Digital Ports and Pins
    GPIO_TypeDef* GL_ActivePort;
    uint16_t GL_ActivePin;
    GPIO_TypeDef* BL_ActivePort;
    uint16_t BL_ActivePin;

    //Debug LED
    GPIO_TypeDef* dbgLedPort;
    uint16_t dbgLedPin;

    //SPI ADC analogue inputs
    ADS7028_HandleTypeDef HSD_ADC1;
    ADS7028_HandleTypeDef HSD_ADC2;
} PDMInfo_t;

extern PDMInfo_t PDM;
#endif /* INC_PDM_H_ */
