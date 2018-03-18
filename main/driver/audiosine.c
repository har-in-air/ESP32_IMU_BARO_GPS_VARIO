#include "common.h"
#include "config.h"

#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"

#include "driver/dac.h"
#include "options.h"
#include "audio.h"


static int clk_8m_div = 7;  // RTC 8M clock divider (division is by clk_8m_div+1, i.e. 0 means 8MHz frequency)
static int frequency_step = 8;  // Frequency step for CW generator
static int scale = 0;           // full scale
static int offset;              // leave it default / 0 = no any offset
static int invert = 2;          // invert MSB to get sine waveform



static void dac_cosine_enable(dac_channel_t channel){
   // Enable tone generator common to both channels
   SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
   switch(channel) {
      case DAC_CHANNEL_1:
      // Enable / connect tone tone generator on / to this channel
      SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
      // Invert MSB, otherwise part of waveform will have inverted
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV1_S);
      break;
   
      case DAC_CHANNEL_2:
      SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
      SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
      break;
      default :
      printf("Channel %d\n", channel);
      }
   }



 // Set frequency of internal CW generator common to both DAC channels 
 // clk_8m_div: 0b000 - 0b111
 // frequency_step: range 0x0001 - 0xFFFF
 
static void dac_frequency_set(int clk_8m_div, int frequency_step){
   REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, clk_8m_div);
   SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, frequency_step, SENS_SW_FSTEP_S);
   }


 // Scale output of a DAC channel using two bit pattern:
 // 00: no scale
 // 01: scale to 1/2
 // 10: scale to 1/4
 // 11: scale to 1/8

void dac_scale_set(dac_channel_t channel, int scale){
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, scale, SENS_DAC_SCALE1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, scale, SENS_DAC_SCALE2_S);
            break;
        default : break;
      }
   }


// Offset output of a DAC channel
// Range 0x00 - 0xFF
void dac_offset_set(dac_channel_t channel, int offset){
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC1, offset, SENS_DAC_DC1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2, offset, SENS_DAC_DC2_S);
            break;
        default : break;
    }
}



 // Invert output pattern of a DAC channel
 // 00: does not invert any bits,
 // 01: inverts all bits,
 // 10: inverts MSB,
 // 11: inverts all bits except for MSB
void dac_invert_set(dac_channel_t channel, int invert){
    switch(channel) {
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, invert, SENS_DAC_INV1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, invert, SENS_DAC_INV2_S);
            break;
        default : break;
    }
}


void audio_config(int pnDacChan) {
   AUDIO_AMP_DISABLE();
   dac_cosine_enable(DAC_CHANNEL_1);
   dac_output_enable(DAC_CHANNEL_1);
	scale =  3 - opt.misc.speakerVolume;
	offset = 0;
	invert = 2;
   dac_scale_set(DAC_CHANNEL_1, scale);
   dac_offset_set(DAC_CHANNEL_1, offset);
   dac_invert_set(DAC_CHANNEL_1, invert);
	}
	
	
void audio_setFrequency(int freqHz) {
	if (freqHz > 0) {
      AUDIO_AMP_ENABLE();
      dac_output_enable(DAC_CHANNEL_1);
	   frequency_step = (freqHz*(1+clk_8m_div)*65536)/RTC_FAST_CLK_FREQ_APPROX;
      dac_frequency_set(clk_8m_div, frequency_step);
		}
	else {
      dac_output_disable(DAC_CHANNEL_1);
      AUDIO_AMP_DISABLE();
		}
	}	
	

void audio_generateTone(int freqHz, int milliseconds) {
	audio_setFrequency(freqHz);
	delayMs(milliseconds);
	audio_setFrequency(0);
	}
	
