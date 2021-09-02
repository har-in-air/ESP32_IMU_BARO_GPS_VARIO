#ifndef BMP_388_H_
#define BMP_388_H_

extern float ZCmAvg_BMP388;
extern float ZCmSample_BMP388;
extern float PaSample_BMP388;
extern int   CelsiusSample_BMP388;

int bmp388_config();
void bmp388_sample();
void bmp388_measure_noise();
void bmp388_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data);
void bmp388_averaged_sample(int nSamples);
float bmp388_pa2Cm(float fpa);

#endif