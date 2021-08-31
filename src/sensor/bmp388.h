#ifndef BMP_388_H_
#define BMP_388_H_

extern float ZCmAvg_BMP388;
extern float ZCmSample_BMP388;
extern float PaSample_BMP388;
extern int   CelsiusSample_BMP388;

int bmp388_config();
void bmp388_sample();
void bmp388_averaged_sample(int nSamples);

#endif