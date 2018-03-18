#ifndef BEEPER_H_
#define BEEPER_H_


typedef struct BEEP_ {
    int periodTicks;  // on-time + off-time
    int endTick; // on-time
} BEEP;

// clamp climbrate/sinkrate for audio feedback to +/- 10 m/s
#define VARIO_MAX_CPS         1000

#define VARIO_STATE_SINK    	11
#define VARIO_STATE_QUIET   	22
#define VARIO_STATE_ZEROES		33
#define VARIO_STATE_CLIMB   	44


void beeper_config();
void beeper_beep(int32_t cps);


#endif
