#ifndef COMMON_H_
#define COMMON_H_

//#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
//#include <esp_err.h>

extern volatile int     LedState;
extern volatile float   KFAltitudeCm, KFClimbrateCps,DisplayClimbrateCps;
extern volatile float   IirClimbrateCps,GlideRatio, GlideRatioNew;
extern volatile float   YawDeg, PitchDeg, RollDeg;

typedef struct KEY_VAL_ {
   char szName[30];
   char szValue[10];
   } KEY_VAL;

// Arduino defines delay() like this
// vTaskDelay((ms) / portTICK_PERIOD_MS)
// So minimum delay == FreeRTOS tick interval
#define delayMs(ms)     delay((ms)) 

#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                   ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)           {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)                {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)               {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		   (((float)((x)-(mn))/(float)((mx)-(mn))) - 0.5f)
#define INTEGER_ROUNDUP(val)     ((val) >= 0.0f ? (int32_t)((val)+0.5f) : (int32_t)((val)-0.5f))

#define RAD2DEG(r)   ((r)*57.29577951f)
#define DEG2RAD(d)   ((d)*0.017453292f)

#define _180_DIV_PI         57.2957795f
#define PI_DIV_180          0.017453292f
#define _2_PI              6.2831853f

#define CORE_0    0
#define CORE_1    1

#endif
