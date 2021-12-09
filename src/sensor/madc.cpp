#include "common.h"
#include "config.h"
#include "madc.h"

static const char* TAG = "madc";

#define NUM_ADC_SAMPLES   4          // averaging 

void adc_init() {
    analogReadResolution(10);
    analogSetAttenuation(ADC_0db); // adc range 0 to 0.8V, use external resistor drop
    adcAttachPin(pinADC);
    }

int adc_sample_average() {
    int sample = 0;
    for (int inx = 0; inx < NUM_ADC_SAMPLES; inx++) {
        sample += analogRead(pinADC); 
        }
    sample /= NUM_ADC_SAMPLES;  
    return sample;
    }

// supply-10K-2K-ground resistor divider
// two point calibration
#define V1 3.247f
#define V2 5.17f
#define ADC1 491.0f
#define ADC2 775.0f

float adc_battery_voltage() {
    int sample = adc_sample_average();
    float slope = (V2 - V1)/(ADC2 - ADC1);
    float voltage = slope*(sample - ADC1) + V1;
    return voltage;
    }
