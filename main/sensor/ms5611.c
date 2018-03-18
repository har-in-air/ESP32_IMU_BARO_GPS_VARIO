#include "common.h"
#include "config.h"
#include "cct.h"
#include "ms5611.h"
#include "vspi.h"

float ZCmAvg;
float ZCmSample;
float PaSample;
int   CelsiusSample;

static uint8_t 	Prom_[16];
static uint16_t   Cal_[6];
static int64_t 	Tref_;
static int64_t 	OffT1_;
static int64_t 	SensT1_;	
static int32_t 	TempCx100_;
static uint32_t   D1_;
static uint32_t   D2_;
static int64_t 	DT_;	
static int 		   SensorState_;

#define TAG "ms5611"


int ms5611_config(void) {
	BARO_CS_HI();
	PaSample = 0.0f;
	ZCmSample = 0.0f;
	CelsiusSample = 0;
	ZCmAvg = 0.0f;
	//ms5611_reset();
	if (!ms5611_readPROM()) {
		ESP_LOGE(TAG, "Error reading calibration PROM");
		return -1;
		}
	ms5611_getCalibrationParameters();
	return 0;
	}


void ms5611_initializeSampleStateMachine(void) {
   ms5611_triggerTemperatureSample();
   SensorState_ = MS5611_READ_TEMPERATURE;
   }


int ms5611_sampleStateMachine(void) {
   if (SensorState_ == MS5611_READ_TEMPERATURE) {
      D2_ = ms5611_readSample();
      ms5611_triggerPressureSample();
      ms5611_calculateTemperatureCx10();
      //CelsiusSample_ = (TempCx100_ >= 0? (TempCx100_+50)/100 : (TempCx100_-50)/100);
      PaSample = ms5611_calculatePressurePa();
	   ZCmSample = ms5611_pa2Cm(PaSample);
	   SensorState_ = MS5611_READ_PRESSURE;
      return 1;  // new altitude sample is available
      }
   else
   if (SensorState_ == MS5611_READ_PRESSURE) {
      D1_ = ms5611_readSample();
      ms5611_triggerTemperatureSample();
      SensorState_ = MS5611_READ_TEMPERATURE;
      return 0; // intermediate state, no new result available
      }
   return 0;    
   }

#ifdef MS5611_TEST

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void ms5611_test(int nSamples) {
	int32_t n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    ms5611_triggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = ms5611_readSample();
	    ms5611_calculateTemperatureCx10();
		ms5611_triggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ms5611_readSample();
		pa[n] = ms5611_calculatePressurePa();
        z[n] =  ms5611_pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        ESP_LOGI(TAG,"%f %f\r\n",pa[n],z[n]);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    ESP_LOGI(TAG,"paVariance %f  zVariance %f",paVariance,zVariance);    
	}
#endif


void ms5611_averagedSample(int nSamples) {
	int32_t tc,tAccum,n;
   float pa,pAccum;
	pAccum = 0.0f;
   tAccum = 0;
	n = nSamples;
   while (n--) {
		ms5611_triggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ms5611_readSample();
		ms5611_calculateTemperatureCx10();
		ms5611_triggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ms5611_readSample();
		pa = ms5611_calculatePressurePa();
		pAccum += pa;
		tAccum += TempCx100_;
		}
	tc = tAccum/nSamples;
	CelsiusSample = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	PaSample = (pAccum+nSamples/2)/nSamples;
	ZCmAvg = ZCmSample = ms5611_pa2Cm(PaSample);
	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.txt"

float ms5611_pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)paf;

   	if (pa > PA_INIT) {
      	zf = (float)(PZTable[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(PZTable[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = PZTable[inx];
         	z2 = PZTable[inx+1];
         	zf = (float)z1 + ((((float)pa1)-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}


void ms5611_triggerPressureSample(void) {
	spiSimpleTransaction(_vspi);
   BARO_CS_LO();
	spiTransferByteNL(_vspi, MS5611_CMD_CONVERT_D1|MS5611_CMD_ADC_4096);
	BARO_CS_HI();
	spiEndTransaction(_vspi);
	}


void ms5611_triggerTemperatureSample(void) {
	spiSimpleTransaction(_vspi);
   BARO_CS_LO();
   spiTransferByteNL(_vspi, MS5611_CMD_CONVERT_D2|MS5611_CMD_ADC_4096);
   BARO_CS_HI();
	spiEndTransaction(_vspi);
	}


uint32_t ms5611_readSample(void)	{
	uint32_t w, b0, b1, b2;
	spiSimpleTransaction(_vspi);
	BARO_CS_LO();
	spiTransferByteNL(_vspi, MS5611_CMD_ADC_READ);
	b0 = (uint32_t)spiTransferByteNL(_vspi, 0);
	b1 = (uint32_t)spiTransferByteNL(_vspi, 0);
	b2 = (uint32_t)spiTransferByteNL(_vspi, 0);
	w = ((b0<<16) | (b1<<8) | b2);
	BARO_CS_HI();
	spiEndTransaction(_vspi);
	return w;
	}


void ms5611_calculateTemperatureCx10(void) {
	DT_ = (int64_t)D2_ - Tref_;
	TempCx100_ = 2000 + ((DT_*((int32_t)Cal_[5]))>>23);
	}


float ms5611_calculatePressurePa(void) {
   float pa;
   int64_t offset, sens,offset2,sens2,t2;
   offset = OffT1_ + ((((int64_t)Cal_[3])*(int64_t)DT_)>>7);
   sens = SensT1_ + ((((int64_t)Cal_[2])*(int64_t)DT_)>>8);
   if (TempCx100_ < 2000) {
      t2 = ((DT_*DT_)>>31); 
      offset2 = (5*(TempCx100_-2000)*(TempCx100_-2000))/2;
      sens2 = offset2/2;
      } 
   else {
      t2 = 0;
      sens2 = 0;
      offset2 = 0;
      }
   TempCx100_ -= t2;
   offset -= offset2;
   sens -= sens2;
   pa = ((float)((int64_t)D1_ * sens)/2097152.0f - (float)(offset)) / 32768.0f;
   return pa;
   }



void ms5611_reset(void) {
	BARO_CS_LO();
	spiTransferByteNL(_vspi, MS5611_CMD_RESET);
	BARO_CS_HI();
	spiEndTransaction(_vspi);
	cct_delayUs(4000); // > 3mS as per app note AN520	
   }

   	
void ms5611_getCalibrationParameters(void)  {
   for (int inx = 0; inx < 6; inx++) {
		int promIndex = 2 + inx*2; 
		Cal_[inx] = (((uint16_t)Prom_[promIndex])<<8) | (uint16_t)Prom_[promIndex+1];
		}
   ESP_LOGI(TAG,"Calib Coeffs : %d %d %d %d %d %d",Cal_[0],Cal_[1],Cal_[2],Cal_[3],Cal_[4],Cal_[5]);
   Tref_ = ((int64_t)Cal_[4])<<8;
   OffT1_ = ((int64_t)Cal_[1])<<16;
   SensT1_ = ((int64_t)Cal_[0])<<15;		
   }
   

int ms5611_readPROM(void)    {
   for (int inx = 0; inx < 8; inx++) {
    	BARO_CS_LO();
		spiTransferByteNL(_vspi, 0xA0 + inx*2);
		Prom_[inx*2] = spiTransferByteNL(_vspi, 0);
		Prom_[inx*2+1] = spiTransferByteNL(_vspi, 0);
		BARO_CS_HI();
		}			
   uint8_t crcPROM = Prom_[15] & 0x0F;
	uint8_t crcCalculated = ms5611_CRC4(Prom_);
	return (crcCalculated == crcPROM ? 1 : 0);
	}
	
	
uint8_t ms5611_CRC4(uint8_t prom[] ) {
   int cnt, nbit; 
   uint16_t crcRemainder; 
   uint8_t crcSave = prom[15]; // crc byte in PROM
   ESP_LOGI(TAG,"PROM CRC = 0x%x", prom[15] & 0x0F);
   crcRemainder = 0x0000;
   prom[15] = 0; //CRC byte is replaced by 0

   for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000; 
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
   crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
   prom[15] = crcSave; // restore the crc byte
   ESP_LOGI(TAG, "Calculated CRC = 0x%x",  crcRemainder ^ 0x0);
   return (uint8_t)(crcRemainder ^ 0x0);
   } 


