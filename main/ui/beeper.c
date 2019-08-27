#include "common.h"
#include "config.h"
#include "options.h"
#include "audio.h"
#include "beeper.h"

#define TAG "beeper"

static int32_t VarioCps_; // internal state : current climb/sink rate
static int32_t FreqPlayingHz_; // internal state : current frequency being generated
// sinktone indicates sinking air, this is a warning tone
static int32_t SinkToneCps_; // threshold in cm per second
// climbtone indicates lift is strong enough to try turning to stay in it
static int32_t ClimbToneCps_; // threshold in cm per second
// zeroestone indicates weak lift, possibility of stronger lift/thermal nearby
static int32_t ZeroesToneCps_; // threshold in cm per second
// allocate roughly 1 decade (10:1) of speaker frequency bandwidth to climbrates below
// crossoverCps, and 1 octave (2:1) of frequency bandwidth to climbrates above
// crossoverCps. So if you are flying in strong conditions, increase crossoverCps.
// If you are flying in weak conditions, decrease crossoverCps.
static int32_t CrossoverCps_;

static int BeepPeriodTicks_; // internal state : current beep interval in ticks
static int BeepEndTick_; // internal state : current beep  on-time in ticks
static int CurrentTick_; // internal state : current tick ( 1 tick ~= 20mS)
static int VarioState_; // internal state : climb/zeroes/quiet/sink
// for offscale climbrates above +10m/s generate continuous warbling tone
static int OffScaleHiTone_[8]  = {400,800,1200,1600,2000,1600,1200,800};
// for offscale sinkrates below -10m/s generate continuous descending tone
static int OffScaleLoTone_[8]  = {4000,3500,3000,2500,2000,1500,1000,500};
// {beep_period, beep_on_time} based on vertical climbrate in 1m/s intervals
static BEEP BeepTbl_[10] = {
{16,10}, // 0m/s to +1m/s
{14,9},  
{12,8},
{10,7},
{9,6},
{8,5},
{7,4},
{6,3},
{5,2},
{4,2}, // +9m/s to +10m/s
};

void beeper_config() {
	SinkToneCps_    = (int32_t)opt.vario.sinkThresholdCps;
	ClimbToneCps_   = (int32_t)opt.vario.climbThresholdCps;
	ZeroesToneCps_  = (int32_t)opt.vario.zeroThresholdCps;
	CrossoverCps_   = (int32_t)opt.vario.crossoverCps;
#ifdef BEEPER_DEBUG  
   ESP_LOGI(TAG,"climbToneCps = %d", ClimbToneCps_);
   ESP_LOGI(TAG,"zeroesToneCps = %d", ZeroesToneCps_);
   ESP_LOGI(TAG,"sinkToneCps = %d", SinkToneCps_);
   ESP_LOGI(TAG,"crossoverCps = %d", CrossoverCps_);
#endif  
   VarioState_ 		= VARIO_STATE_QUIET;
	BeepPeriodTicks_	= 0;
	BeepEndTick_ 		= 0;
	VarioCps_ 			= 0;
	FreqPlayingHz_  	= 0;
   }


void beeper_beep(int32_t nCps) {
  int32_t newFreqHz = 0;
  // generate new beep/tone only if 
  if (
    // current beep/tone has ended, OR
    (BeepPeriodTicks_ <= 0)  ||
#ifdef VARIO_INTERRUPT_BEEPS    
    // at least half current beep/tone is over AND there is a significant change in climb/sink, OR
    ((CurrentTick_ >= BeepPeriodTicks_/2) && (ABS(nCps - VarioCps_) > VARIO_DISCRIMINATION_THRESHOLD_CPS)) || 
#endif    
    // climb threshold exceeded
    ((nCps >= ClimbToneCps_) && (VarioCps_ < ClimbToneCps_)) 
     ) {
    VarioCps_ = nCps;
    // if sinking significantly faster than glider sink rate in still air, generate warning sink tone
    if (VarioCps_ <= SinkToneCps_) {
      VarioState_ = VARIO_STATE_SINK;
			CurrentTick_ = 0;
      if (VarioCps_ <= -VARIO_MAX_CPS) {
        BeepPeriodTicks_ = 8;
        BeepEndTick_ = 8;
    		newFreqHz = OffScaleLoTone_[0];
    		FreqPlayingHz_ = newFreqHz;
        audio_setFrequency(FreqPlayingHz_);
        }
      else {
        BeepPeriodTicks_ = 40; // sink indicated with descending frequency beeps with long on-times
        BeepEndTick_  = 30;
        // descending tone starts at higher frequency for higher sink rate
        newFreqHz = VARIO_SPKR_MAX_FREQHZ/2 + ((VarioCps_ + VARIO_MAX_CPS)*(VARIO_SPKR_MIN_FREQHZ + 600 - VARIO_SPKR_MAX_FREQHZ/2))/(SinkToneCps_ + VARIO_MAX_CPS);
        CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
        FreqPlayingHz_ = newFreqHz;
        audio_setFrequency(FreqPlayingHz_);
        }
      }
    //if climbing, generate beeps
    else {
      if (VarioCps_ >= ClimbToneCps_) {
        VarioState_ = VARIO_STATE_CLIMB;
        CurrentTick_ = 0;
        if (VarioCps_ >= VARIO_MAX_CPS) {
          BeepPeriodTicks_ = 8;
          BeepEndTick_ = 8;
          newFreqHz = OffScaleHiTone_[0];
          FreqPlayingHz_ = newFreqHz;
          audio_setFrequency(FreqPlayingHz_);
          }
        else {
          int index = VarioCps_/100;
          if (index > 9) index = 9;
          BeepPeriodTicks_ = BeepTbl_[index].periodTicks;
          BeepEndTick_ = BeepTbl_[index].endTick;
          if (VarioCps_ > CrossoverCps_) {
            newFreqHz = VARIO_CROSSOVER_FREQHZ + ((VarioCps_ - CrossoverCps_)*(VARIO_SPKR_MAX_FREQHZ - VARIO_CROSSOVER_FREQHZ))/(VARIO_MAX_CPS - CrossoverCps_);
            }
          else {
            newFreqHz = VARIO_SPKR_MIN_FREQHZ + ((VarioCps_ - ZeroesToneCps_)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CrossoverCps_ - ZeroesToneCps_);
            }
          CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
          FreqPlayingHz_ = newFreqHz;
          audio_setFrequency(FreqPlayingHz_);
          }
        }
      else   // in "zeroes" band, indicate with a short pulse and long interval
      if (VarioCps_ >= ZeroesToneCps_) {
        VarioState_ = VARIO_STATE_ZEROES;
    		CurrentTick_ = 0;
    		BeepPeriodTicks_ = 30;
    		BeepEndTick_ = 5;
    		newFreqHz = VARIO_SPKR_MIN_FREQHZ + ((VarioCps_ - ZeroesToneCps_)*(VARIO_CROSSOVER_FREQHZ - VARIO_SPKR_MIN_FREQHZ))/(CrossoverCps_ - ZeroesToneCps_);
        CLAMP(newFreqHz, VARIO_SPKR_MIN_FREQHZ, VARIO_SPKR_MAX_FREQHZ);
        FreqPlayingHz_ = newFreqHz;
        audio_setFrequency(FreqPlayingHz_);
        }
      // between zeroes threshold and sink threshold, chillout
      else{
        VarioState_ = VARIO_STATE_QUIET;
        CurrentTick_ = 0;
        BeepPeriodTicks_ = 0;
        BeepEndTick_  = 0;
        FreqPlayingHz_ = 0;
        audio_setFrequency(FreqPlayingHz_);
        }
      }
    }
  else { // still processing current beep/tone
    CurrentTick_++;
    BeepPeriodTicks_--;
    if (CurrentTick_ >= BeepEndTick_){ // shut off climb beep after 'on' time ends
      newFreqHz = 0;
      }
	  else
	  if (VarioCps_ >= VARIO_MAX_CPS) { // offscale climbrate (>= +10m/s) indicated with continuous warbling tone
      newFreqHz = OffScaleHiTone_[CurrentTick_];
      }
    else
	  if (VarioCps_ <= -VARIO_MAX_CPS) {  // offscale sink (<= -10m/s) indicated with continuous descending tone
      newFreqHz = OffScaleLoTone_[CurrentTick_];
      }
    else
	  if (VarioState_ == VARIO_STATE_SINK) {  // sink is indicated with a descending frequency beep
      newFreqHz = FreqPlayingHz_ - 20;
      }
    else {
      newFreqHz = FreqPlayingHz_; // no change   
      }
    if (newFreqHz != FreqPlayingHz_) {
      FreqPlayingHz_ = newFreqHz;
      audio_setFrequency(FreqPlayingHz_);
      }
	  }
  }


