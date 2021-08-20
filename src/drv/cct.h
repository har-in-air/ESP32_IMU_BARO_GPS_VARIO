#ifndef CCT_H_
#define CCT_H_

#include "xtensa/core-macros.h"
#include "sdkconfig.h"

#define CCT_TICKS_PER_US 	CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 
#define cct_setMarker()     XTHAL_GET_CCOUNT()

void        cct_delayUs(uint32_t us);
uint32_t    cct_intervalUs(uint32_t before, uint32_t after);
float       cct_intervalSecs(uint32_t before, uint32_t after);
uint32_t    cct_elapsedUs(uint32_t clockPrev);

#endif
