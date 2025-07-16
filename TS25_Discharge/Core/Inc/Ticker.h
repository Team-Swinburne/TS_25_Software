/*
 * Ticker.h
 *
 *  Created on: Jul 27, 2024
 *      Author: Ethan Jones
 */

#ifndef INC_TICKER_H_
#define INC_TICKER_H_

//Libraries
#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#define MAX_NUMBER_OF_TICKERS 10

typedef void (*fpointer)();

typedef struct tickerInfo {
    uint8_t tickers;
    uint32_t counter[MAX_NUMBER_OF_TICKERS];
    uint32_t intervals[MAX_NUMBER_OF_TICKERS];
    fpointer callbackFunctions[MAX_NUMBER_OF_TICKERS];
} TickerInfo_t;

extern TickerInfo_t Ticker;

//Functions
void TickerAttach(TickerInfo_t*, fpointer, uint32_t);
void TickerHandler(TickerInfo_t*);

#endif /* INC_TICKER_H_ */
