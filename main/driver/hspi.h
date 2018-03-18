#ifndef LCDSPI_H_
#define LCDSPI_H_

#include "Arduino.h"

void hspi_config(int pnSCLK, int pnMOSI, int pnMISO, int freqHz);
void hspi_setClockFreq(int freqHz);

extern spi_t* _hspi;


#endif
