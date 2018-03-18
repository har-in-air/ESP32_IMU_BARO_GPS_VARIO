#ifndef MYADC_H_
#define MYADC_H_

// 10k+3k resistor divider,measured 0.949V with a battery voltage of 4.1V
#define adc_batteryVoltage()   ((uint32_t)((4.1f/0.949f)*adc_sample()))

uint32_t adc_sample(void);

#endif
