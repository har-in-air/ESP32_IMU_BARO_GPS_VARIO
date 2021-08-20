#include "common.h"
#include "cct.h"

static const char* TAG = "cct";

// only works if task is locked to one cpu

void cct_delayUs(uint32_t us) {
  volatile uint32_t waitCycleCount = (CCT_TICKS_PER_US * us ) + XTHAL_GET_CCOUNT();
  do  {} while (XTHAL_GET_CCOUNT()  < waitCycleCount);
  }


uint32_t  cct_intervalUs(uint32_t before, uint32_t after) {
   return  (before <= after ?
      ((after - before)+CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
      (after + (0xFFFFFFFF - before) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
   }


float  cct_intervalSecs(uint32_t before, uint32_t after) {
   return  (before <= after ?
      (float)(after - before)/(float)(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ*1000000) :
      (float)(after + (0xFFFFFFFF - before))/(float)(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ*1000000));
   }


uint32_t cct_elapsedUs(uint32_t clockPrev) {
   uint32_t clockNow = XTHAL_GET_CCOUNT();
   return  (clockPrev <= clockNow ?
      ((clockNow - clockPrev) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US :
      (clockNow + (0xFFFFFFFF - clockPrev) + CCT_TICKS_PER_US/2)/CCT_TICKS_PER_US);
   }
