#ifndef MADC_H_
#define MADC_H_

// supply-10K-2K-ground resistor divider, measured 0.823V with a supply voltage of 4.93V
#define adc_supplyVoltageMV()   ((uint32_t)((4.93f/0.823f)*adc_sampleMV()))

void adc_init();
uint32_t adc_sampleMV(void);

#endif
