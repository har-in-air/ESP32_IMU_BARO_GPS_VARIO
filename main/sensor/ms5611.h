#ifndef MS5611_H_
#define MS5611_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define MS5611_TEST

// 10mS enough, but we're using vTaskDelay with a tick period of 10mS
// so 2 ticks are required to ensure at least one tick delay !
#define MS5611_SAMPLE_PERIOD_MS         20

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22

#define MS5611_CMD_RESET      	0x1E
#define MS5611_CMD_CONVERT_D1 	0x40
#define MS5611_CMD_CONVERT_D2 	0x50
#define MS5611_CMD_ADC_READ   	0x00
#define MS5611_CMD_ADC_4096 	0x08

void 	ms5611_triggerPressureSample(void);
void 	ms5611_triggerTemperatureSample(void);
uint32_t  	ms5611_readSample(void);
void 	ms5611_averagedSample(int nSamples);	
void  	ms5611_calculateTemperatureC(void);
float  	ms5611_calculatePressurePa(void);
void 	ms5611_calculateSensorNoisePa(void);
int 	ms5611_config(void);
int 	ms5611_sampleStateMachine(void);
void 	ms5611_initializeSampleStateMachine(void);
float  	ms5611_pa2Cm(float pa);
void 	ms5611_test(int nSamples);	
uint8_t ms5611_CRC4(uint8_t prom[] );
int 	ms5611_readPROM(void);
void 	ms5611_getCalibrationParameters(void);
void 	ms5611_reset(void);

extern	float ZCmAvg;
extern	float ZCmSample;
extern	float PaSample;
extern	int   CelsiusSample;

#ifdef __cplusplus
}
#endif
#endif // MS5611_H_
