#ifndef KALMAN_FILTER3_H_
#define KALMAN_FILTER3_H_

void kalmanFilter3_configure(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial);
void kalmanFilter3_predict(float a, float dt);
void kalmanFilter3_update(float z, float* pZ, float* pV);

#endif

