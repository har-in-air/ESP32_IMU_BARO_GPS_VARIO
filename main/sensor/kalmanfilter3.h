#ifndef KALMAN_FILTER3_H_
#define KALMAN_FILTER3_H_

#ifdef __cplusplus
extern "C" {
#endif

void kalmanFilter3_configure(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial);
void kalmanFilter3_update(float z, float a, float dt, float* pZ, float* pV);

#ifdef __cplusplus
}
#endif

#endif

