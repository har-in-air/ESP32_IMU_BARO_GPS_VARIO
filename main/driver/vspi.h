#ifndef SPIHW_H_
#define SPIHW_H_

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

void vspi_config(int pnSCLK, int pnMOSI, int pnMISO, int freqHz);
void vspi_setClockFreq(int freqHz);

extern spi_t* _vspi;

#ifdef __cplusplus
}
#endif

#endif
