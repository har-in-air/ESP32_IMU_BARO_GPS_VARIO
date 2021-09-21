
#ifndef KALMAN_FILTER2_H_
#define KALMAN_FILTER2_H_

void kalmanFilter2_configure(float zVariance, float zAccelVariance,  float zInitial, float vInitial);
void kalmanFilter2_predict(float zAccelVariance,float dt);
void kalmanFilter2_update(float z, float* pZ, float* pV);


#endif
