/*
 * Ticker.c
 *
 *  Created on: Jul 27, 2024
 *      Author: Ethan Jones
 */

#include "Ticker.h"

TickerInfo_t Ticker = {-1, 0, 0, NULL};

void TickerAttach(TickerInfo_t* Ticker, fpointer FunctionCallback, uint32_t interval)
{
    if (Ticker->tickers == MAX_NUMBER_OF_TICKERS - 1) return; //Check if max number of tickers have been reached.

    Ticker->tickers++;
    Ticker->intervals[Ticker->tickers] = interval;
    Ticker->callbackFunctions[Ticker->tickers] = FunctionCallback;
}

void TickerHandler(TickerInfo_t* Ticker)
{
    for (int i = 0; i < Ticker->tickers + 1; i++)
    {
        Ticker->counter[i]++;

        if (Ticker->counter[i]/ Ticker->intervals[i] >= 1)
        {
            Ticker->counter[i] = 0;
            Ticker->callbackFunctions[i]();
        }
    }
}
