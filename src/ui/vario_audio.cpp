#include "common.h"
#include "config.h"
#include "nv/options.h"
#include "drv/audio.h"
#include "vario_audio.h"

static const char* TAG = "vario_audio";
// indicate climb/sink rates beyond + / - 10m/s with offscale warbling tones
#define VARIO_MAX_CPS         1000

// a beep is an interval with a frequency tone (on-time)
// followed by a silent interval (off-time)
typedef struct BEEP_ {
    int periodTicks;  // on-time + off-time
    int endTick; // on-time
} BEEP;

static const BEEP BeepTbl[10] = {
    {16,10}, // 0m/s - 1m/s
    {14,9},  // 1m/s - 2m/s
    {12,8},
    {10,7},
    {9,6},
    {8,5},
    {7,4},
    {6,3},
    {5,2},
    {4,2}, // +9m/s to +10m/s
    };	

static int Tick;
static int BeepEndTick;
static int BeepPeriodTicks;
static int32_t CurrentCps;
static int CurrentFreqHz;

#define CROSSOVER_CPS	((int32_t)opt.vario.crossoverCps)
#define ZERO_CPS		((int32_t)opt.vario.zeroThresholdCps)
#define SINK_CPS		((int32_t)opt.vario.sinkThresholdCps)
#define CLIMB_CPS		((int32_t)opt.vario.climbThresholdCps)
          

// for offscale climbrates above +10m/s generate continuous warbling tone
// this table contains the frequencies to use at each tick
static const int OffScaleHiTone[8]  = {400,800,1200,1600,2000,1600,1200,800};
// for offscale sinkrates below -10m/s generate continuous descending tone
static const int OffScaleLoTone[8]  = {4000,3500,3000,2500,2000,1500,1000,500};

void vaudio_config() {
	Tick = 0;
	BeepPeriodTicks = 0;
    BeepEndTick		= 0;
    CurrentCps		= 0;
    CurrentFreqHz	= 0;
    }


static void vaudio_reset(int32_t cps) {
	CurrentCps = cps;
	Tick = 0;
	if (CurrentCps <= SINK_CPS) {
		if (CurrentCps <= -VARIO_MAX_CPS) {
			// off-scale sink warbling tone
			BeepPeriodTicks = 8;
			BeepEndTick = 8;
			CurrentFreqHz = OffScaleLoTone[0];
			audio_set_frequency(CurrentFreqHz);
			}
		else {
			// sink indicated with descending frequency beeps with long on-times
			// beep starts at higher frequency for higher sink rate
			BeepPeriodTicks = 40; 
			BeepEndTick  = 30;
			CurrentFreqHz = VARIO_SPKR_MAX_FREQHZ/2 + ((CurrentCps + VARIO_MAX_CPS)*(VARIO_SPKR_MIN_FREQHZ + 600 - VARIO_SPKR_MAX_FREQHZ/2))/(SINK_CPS + VARIO_MAX_CPS);
			CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
			audio_set_frequency(CurrentFreqHz);
			}
        }
    else {
		if (CurrentCps >= CLIMB_CPS) {
			if (CurrentCps >= VARIO_MAX_CPS) {
				// off-scale climb warbling tone
				BeepPeriodTicks = 8;
				BeepEndTick = 8;
				CurrentFreqHz = OffScaleHiTone[0];
				audio_set_frequency(CurrentFreqHz);
				}
			else {
				// climb beep period reduces with higher climbrate
				// beep tone frequency increases with higher climbrate
				int index = CurrentCps/100;
				if (index > 9) index = 9;
				BeepPeriodTicks = BeepTbl[index].periodTicks;
				BeepEndTick = BeepTbl[index].endTick;
				if (CurrentCps > CROSSOVER_CPS) {
					CurrentFreqHz = VARIO_CROSSOVER_FREQHZ + ((CurrentCps - CROSSOVER_CPS)*(VARIO_SPKR_MAX_FREQHZ - VARIO_CROSSOVER_FREQHZ))/(VARIO_MAX_CPS - CROSSOVER_CPS);
					}
				else {
					CurrentFreqHz = VARIO_SPKR_MIN_FREQHZ + ((CurrentCps - ZERO_CPS)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CROSSOVER_CPS - ZERO_CPS);
					}
				CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
				audio_set_frequency(CurrentFreqHz);
				}
			}
		else 
		if (CurrentCps >= ZERO_CPS) {
			// "zeros" band, beep is a short pulse and long interval
			BeepPeriodTicks = 30;
			BeepEndTick = 2;
			CurrentFreqHz = VARIO_SPKR_MIN_FREQHZ + ((CurrentCps - ZERO_CPS)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CROSSOVER_CPS - ZERO_CPS);
			CLAMP(CurrentFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
			audio_set_frequency(CurrentFreqHz);
			}            
		else {
			// between sink and zero threshold, be quiet
			BeepPeriodTicks = 0;
			BeepEndTick  = 0;
			CurrentFreqHz = 0;
			audio_set_frequency(CurrentFreqHz);
			}
		}
	}


void vaudio_tick_handler(int32_t cps) {
	int freqHz;
	if (Tick >= BeepPeriodTicks) {  
		// finished processing current beep/tone
		vaudio_reset(cps);
		}
	else 
	if ((cps >= CLIMB_CPS) && (CurrentCps < CLIMB_CPS)) {		
		// allowed to interrupt current processing to start new beep
		vaudio_reset(cps);
		}
	else {
		// continue to process current beep/tone
		Tick++;
		if (Tick > BeepEndTick) {
			// stay quiet for rest of beep period
			freqHz = 0;
			}
		else
		if (CurrentCps >= VARIO_MAX_CPS) {
			// max climb warbling tone
			freqHz = OffScaleHiTone[Tick];
			}
		else
		if (CurrentCps <= -VARIO_MAX_CPS) {
			// max sink warbling tone
			freqHz = OffScaleLoTone[Tick];			
			}
		else
		if (CurrentCps < SINK_CPS) {
			// sink descending tone
			freqHz = CurrentFreqHz - 20;
			}
		else {
			// no change
			freqHz = CurrentFreqHz;
			}
		if (freqHz != CurrentFreqHz) {
			CurrentFreqHz = freqHz;
			audio_set_frequency(CurrentFreqHz);
			}
		}
	}
 