#ifndef MYADC_H_
#define MYADC_H_

#ifdef __cplusplus
extern "C" {
#endif

// supply-10K+2K-ground resistor divider, measured 0.823V with a supply voltage of 4.93V
#define adc_supplyVoltage()   ((uint32_t)((4.93f/0.823f)*adc_sample()))

void adc_init();
uint32_t adc_sample(void);

#ifdef __cplusplus
}
#endif

#endif
