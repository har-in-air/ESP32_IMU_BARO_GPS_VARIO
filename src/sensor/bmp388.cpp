#include "common.h"
#include "config.h"
#include "drv/vspi.h"
#include "drv/cct.h"
#include "bmp3_defs.h"
#include "bmp388.h"

static const char* TAG = "bmp388";

float ZCmAvg_BMP388;
float ZCmSample_BMP388;
float PaSample_BMP388;
int   CelsiusSample_BMP388;

#define BMP388_SPI_READ      ((uint8_t)0x80)
#define BMP388_CHIP_ID       ((uint8_t)0x50)

// credits : adapted from https://github.com/BoschSensortec/BMP3-Sensor-API, using float instead of double, removed pow()

struct bmp3_dev dev;

static void bmp388_write_register(uint8_t regAddress, uint8_t data);
static uint8_t bmp388_read_register(uint8_t regAddress);
static void bmp388_read_registers(uint8_t regAddress, uint8_t* dest, uint8_t count);
static void bmp388_read_calib_data(struct bmp3_dev *dev);
static void bmp388_parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev);
static void bmp388_parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data);
static void bmp388_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data, struct bmp3_dev *dev);
static void bmp388_compensate_temperature(float *temperature,
                                     const struct bmp3_uncomp_data *uncomp_data,
                                     struct bmp3_calib_data *calib_data);
static void bmp388_compensate_pressure(float *pressure,
                                  const struct bmp3_uncomp_data *uncomp_data,
                                  const struct bmp3_calib_data *calib_data);
static void bmp388_compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data);
float bmp388_pa2Cm(float paf);


static void bmp388_write_register(uint8_t regAddress, uint8_t data){
	spiSimpleTransaction(_vspi);
	BMP388_CS_LO();
	spiTransferByteNL(_vspi, regAddress);
	spiTransferByteNL(_vspi, data);
	BMP388_CS_HI();
	spiEndTransaction(_vspi);
    }

static void bmp388_read_registers(uint8_t regAddress,  uint8_t* dest, uint8_t count){
	spiSimpleTransaction(_vspi);
	BMP388_CS_LO();
	spiTransferByteNL(_vspi, regAddress | BMP388_SPI_READ);
    spiTransferByteNL(_vspi, 0); // dummy byte
    spiTransferBytesNL(_vspi, NULL, dest, count);
	BMP388_CS_HI();
	spiEndTransaction(_vspi);	
    }

static uint8_t bmp388_read_register(uint8_t regAddress){
	spiSimpleTransaction(_vspi);
	BMP388_CS_LO();
	spiTransferByteNL(_vspi, regAddress | BMP388_SPI_READ);
    spiTransferByteNL(_vspi, 0); // dummy byte
    uint8_t data = spiTransferByteNL(_vspi, 0);
	BMP388_CS_HI();
	spiEndTransaction(_vspi);	
    return data;  
    }


static void bmp388_read_calib_data(struct bmp3_dev *dev){
    uint8_t reg_addr = BMP3_REG_CALIB_DATA;
    uint8_t calib_data[BMP3_LEN_CALIB_DATA] = { 0 };
    bmp388_read_registers(BMP3_REG_CALIB_DATA, calib_data, BMP3_LEN_CALIB_DATA);
    bmp388_parse_calib_data(calib_data, dev);
    }


static void bmp388_parse_calib_data(const uint8_t *reg_data, struct bmp3_dev *dev){
    /* Temporary variable to store the aligned trim data */
    struct bmp3_reg_calib_data *reg_calib_data = &dev->calib_data.reg_calib_data;
    struct bmp3_quantized_calib_data *quantized_calib_data = &dev->calib_data.quantized_calib_data;
    float temp_var;
    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((float)reg_calib_data->par_t1 / temp_var);
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = 1073741824.0f;
    quantized_calib_data->par_t2 = ((float)reg_calib_data->par_t2 / temp_var);
    reg_calib_data->par_t3 = (int8_t)reg_data[4];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_t3 = ((float)reg_calib_data->par_t3 / temp_var);
    reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
    temp_var = 1048576.0f;
    quantized_calib_data->par_p1 = ((float)(reg_calib_data->par_p1 - (16384)) / temp_var);
    reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
    temp_var = 536870912.0f;
    quantized_calib_data->par_p2 = ((float)(reg_calib_data->par_p2 - (16384)) / temp_var);
    reg_calib_data->par_p3 = (int8_t)reg_data[9];
    temp_var = 4294967296.0f;
    quantized_calib_data->par_p3 = ((float)reg_calib_data->par_p3 / temp_var);
    reg_calib_data->par_p4 = (int8_t)reg_data[10];
    temp_var = 137438953472.0f;
    quantized_calib_data->par_p4 = ((float)reg_calib_data->par_p4 / temp_var);
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantized_calib_data->par_p5 = ((float)reg_calib_data->par_p5 / temp_var);
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = 64.0f;
    quantized_calib_data->par_p6 = ((float)reg_calib_data->par_p6 / temp_var);
    reg_calib_data->par_p7 = (int8_t)reg_data[15];
    temp_var = 256.0f;
    quantized_calib_data->par_p7 = ((float)reg_calib_data->par_p7 / temp_var);
    reg_calib_data->par_p8 = (int8_t)reg_data[16];
    temp_var = 32768.0f;
    quantized_calib_data->par_p8 = ((float)reg_calib_data->par_p8 / temp_var);
    reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p9 = ((float)reg_calib_data->par_p9 / temp_var);
    reg_calib_data->par_p10 = (int8_t)reg_data[19];
    temp_var = 281474976710656.0f;
    quantized_calib_data->par_p10 = ((float)reg_calib_data->par_p10 / temp_var);
    reg_calib_data->par_p11 = (int8_t)reg_data[20];
    temp_var = 36893488147419103232.0f;
    quantized_calib_data->par_p11 = ((float)reg_calib_data->par_p11 / temp_var);
    }


static void bmp388_parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data){
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    data_xlsb = (uint32_t)reg_data[0];
    data_lsb = (uint32_t)reg_data[1] << 8;
    data_msb = (uint32_t)reg_data[2] << 16;
    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    data_xlsb = (uint32_t)reg_data[3];
    data_lsb = (uint32_t)reg_data[4] << 8;
    data_msb = (uint32_t)reg_data[5] << 16;
    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
    }

static void bmp388_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data, struct bmp3_dev *dev) {
    uint8_t reg_data[BMP3_LEN_P_T_DATA] = { 0 };
    struct bmp3_uncomp_data uncomp_data = { 0 };
    bmp388_read_registers(BMP3_REG_DATA, reg_data, BMP3_LEN_P_T_DATA);
    bmp388_parse_sensor_data(reg_data, &uncomp_data);
    bmp388_compensate_data(sensor_comp, &uncomp_data, comp_data, &dev->calib_data);
    }

static void bmp388_compensate_temperature(float *temperature,
                                     const struct bmp3_uncomp_data *uncomp_data,
                                     struct bmp3_calib_data *calib_data){
    int64_t uncomp_temp = uncomp_data->temperature;
    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - calib_data->quantized_calib_data.par_t1);
    partial_data2 = (float)(partial_data1 * calib_data->quantized_calib_data.par_t2);

    /* Update the compensated temperature in calib structure since this is
     * needed for pressure calculation */
    calib_data->quantized_calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) *
                                             calib_data->quantized_calib_data.par_t3;

    /* Returns compensated temperature */
    if (calib_data->quantized_calib_data.t_lin < BMP3_MIN_TEMP_FLOAT){
        calib_data->quantized_calib_data.t_lin = BMP3_MIN_TEMP_FLOAT;
        }

    if (calib_data->quantized_calib_data.t_lin > BMP3_MAX_TEMP_FLOAT) {
        calib_data->quantized_calib_data.t_lin = BMP3_MAX_TEMP_FLOAT;
        }

    (*temperature) = calib_data->quantized_calib_data.t_lin;
    }


/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 * For e.g. returns pressure in Pascal p = 95305.295
 */
static void bmp388_compensate_pressure(float *pressure,
                                  const struct bmp3_uncomp_data *uncomp_data,
                                  const struct bmp3_calib_data *calib_data){
    const struct bmp3_quantized_calib_data *quantized_calib_data = &calib_data->quantized_calib_data;
    float comp_press;
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    float tlin = quantized_calib_data->t_lin;
    float tlinpow2 = tlin*tlin;
    float tlinpow3 = tlinpow2*tlin;
    float pres = (float)uncomp_data->pressure;
    float prespow2 = pres*pres;
    float prespow3 = prespow2*pres;

    partial_data1 = quantized_calib_data->par_p6 * tlin;
    partial_data2 = quantized_calib_data->par_p7 * tlinpow2;
    partial_data3 = quantized_calib_data->par_p8 * tlinpow3;
    partial_out1 = quantized_calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = quantized_calib_data->par_p2 * tlin;
    partial_data2 = quantized_calib_data->par_p3 * tlinpow2;
    partial_data3 = quantized_calib_data->par_p4 * tlinpow3;
    partial_out2 = uncomp_data->pressure *
                   (quantized_calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = prespow2;
    partial_data2 = quantized_calib_data->par_p9 + quantized_calib_data->par_p10 * quantized_calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + prespow3 * quantized_calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    if (comp_press < BMP3_MIN_PRES_FLOAT) {
        comp_press = BMP3_MIN_PRES_FLOAT;
        }

    if (comp_press > BMP3_MAX_PRES_FLOAT) {
        comp_press = BMP3_MAX_PRES_FLOAT;
        }

    (*pressure) = comp_press;
    }


static void bmp388_compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data){
    if (sensor_comp == BMP3_PRESS_TEMP) {
        /* NOTE : Temperature compensation must be done first.
        * Followed by pressure compensation
        * Compensated temperature updated in calib structure,
        * is needed for pressure calculation
        */
        bmp388_compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);
        bmp388_compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);            
        }
    else if (sensor_comp == BMP3_PRESS){
        /* NOTE : Temperature compensation must be done first.
        * Followed by pressure compensation
        * Compensated temperature updated in calib structure,
        * is needed for pressure calculation.
        * As only pressure is enabled in 'sensor_comp', after calculating
        * compensated temperature, assign it to zero.
        */
        bmp388_compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);
        comp_data->temperature = 0;
        bmp388_compensate_pressure(&comp_data->pressure, uncomp_data, calib_data);
        }
    else if (sensor_comp == BMP3_TEMP){
        bmp388_compensate_temperature(&comp_data->temperature, uncomp_data, calib_data);
        comp_data->pressure = 0;
        }
    else {
        comp_data->pressure = 0;
        comp_data->temperature = 0;
        }
    }

#if 0
#define MAX_TEST_SAMPLES 32

static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void bmp388_measure_noise(int nSamples) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    struct bmp3_data sample;
    for (n = 0; n < nSamples; n++) {
        delayMs(20);
        bmp388_get_sensor_data(BMP3_PRESS_TEMP, &sample, &dev);
	    pa[n] = (float)sample.pressure;
        z[n] =  bmp388_pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    ESP_LOGD(TAG,"paMean = %fPa  zMean = %fcm", paMean, zMean);
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    ESP_LOGD(TAG,"paVariance %f  zVariance %f cm^2",paVariance,zVariance);    
	}
#endif


void bmp388_averaged_sample(int nSamples) {
    struct bmp3_data sample;
    float pa_average = 0.0f;
    float temp_average = 0.0f;
    for (int n = 0; n < nSamples; n++) {
        delayMs(20);
        bmp388_get_sensor_data(BMP3_PRESS_TEMP, &sample, &dev);
	    pa_average += (float)sample.pressure;
        temp_average += (float)sample.temperature;
        }
    pa_average /= nSamples;
    temp_average /= nSamples;
    ZCmAvg_BMP388 =  bmp388_pa2Cm(pa_average);
    CelsiusSample_BMP388 = temp_average;
    }


/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.txt"

float bmp388_pa2Cm(float paf)  {
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


int bmp388_config() {
    bmp388_write_register(BMP3_REG_CMD, BMP3_SOFT_RESET);
    delayMs(5);
    uint8_t chipid = bmp388_read_register(BMP3_REG_CHIP_ID);
    if (chipid != BMP388_CHIP_ID) {
        ESP_LOGE(TAG, "BMP388 chip id [%d] != 0x%X", chipid, BMP388_CHIP_ID);
        return -1;
        }
    bmp388_read_calib_data(&dev);
    // power on defaults : FIFO is disabled, SPI 4-wire interface, interrupts disabled
    // configure : 8x pressure, 1x temperature oversampling, IIR coeff = 2, 50Hz ODR, normal mode
    bmp388_write_register(BMP3_REG_OSR, BMP3_OVERSAMPLING_8X | (BMP3_NO_OVERSAMPLING << 3));
    bmp388_write_register(BMP3_REG_ODR, BMP3_ODR_50_HZ);
    bmp388_write_register(BMP3_REG_CONFIG, BMP3_IIR_FILTER_COEFF_1 << 1 );
    // pressure and temperature measurement enabled, normal mode
    bmp388_write_register(BMP3_REG_PWR_CTRL, 0x3 | (0x3 << 4));
    //bmp388_measure_noise(40);
    return 0;
    }


void bmp388_sample() {
    struct bmp3_data sample;
    //uint32_t marker =  cct_setMarker();
    bmp388_get_sensor_data(BMP3_PRESS_TEMP, &sample, &dev);
    ZCmSample_BMP388 = bmp388_pa2Cm(sample.pressure);
    CelsiusSample_BMP388 = sample.temperature;
    //uint32_t eus = cct_elapsedUs(marker);
    //ESP_LOGD(TAG, "P = %lf, T = %lf", sample.pressure, sample.temperature);
    //ESP_LOGD(TAG, "Z = %.0f, %dus", bmp388_pa2Cm((float)sample.pressure), eus);
    }