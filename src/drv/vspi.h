#ifndef SPIHW_H_
#define SPIHW_H_

#include <Arduino.h>

void vspi_config(int pnSCLK, int pnMOSI, int pnMISO, int freqHz);
void vspi_setClockFreq(int freqHz);

extern spi_t* _vspi;

#endif
