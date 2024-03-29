#ifndef KALMAN_FILTER3_H_
#define KALMAN_FILTER3_H_

void kalmanFilter3_configure(float zSensorVariance, float aVariance, float zInitial, float vInitial);
void kalmanFilter3_predict(float a, float dt);
void kalmanFilter3_update(float zm, float* pz, float* pv);

#endif

