#ifndef KALMAN_FILTER4_H_
#define KALMAN_FILTER4_H_

void kalmanFilter4_configure(float aVariance, float zInitial, float vInitial, float aInitial);
void kalmanFilter4_predict(float dt);
void kalmanFilter4_update(float zm, float am, float* pz, float* pv);

#endif

