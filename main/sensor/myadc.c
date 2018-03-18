#include "common.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "myadc.h"

#define TAG "adc"

//Note: Different ESP32 modules may have different reference voltages varying from
// 1000mV to 1200mV. 
#define V_REF   1100
#define ADC1_CHANNEL (ADC1_CHANNEL_7)      //GPIO 35

uint32_t adc_sample(void){
    esp_adc_cal_characteristics_t characteristics;
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_0);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_10, &characteristics);
	uint32_t milliVolts = 0;
	for (int inx = 0; inx < 4; inx++) {
		milliVolts += adc1_to_voltage(ADC1_CHANNEL, &characteristics);
		}
	milliVolts /= 4;
	return milliVolts;
	}
