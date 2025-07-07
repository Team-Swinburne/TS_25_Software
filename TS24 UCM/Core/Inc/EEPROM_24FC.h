/*
 * EEPROM_24FC.h
 *
 *  Created on: Aug 7, 2024
 *      Author: Ethan Jones
 */

#ifndef INC_EEPROM_24FC_H_
#define INC_EEPROM_24FC_H_

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define EEPROM_24FC_I2C_TIMEOUT 10

void EEPROM_24FC_Init(I2C_HandleTypeDef *Handle);
_Bool EEPROM_24FC_Poll();
void EEPROM_24FC_ByteWrite(uint16_t WordAddress, uint8_t Data);
uint8_t EEPROM_24FC_ByteRead(uint16_t WordAddress);

#endif /* INC_EEPROM_24FC_H_ */
