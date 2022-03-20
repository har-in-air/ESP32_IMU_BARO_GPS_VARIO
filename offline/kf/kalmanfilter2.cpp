#include "common.h"
#include "config.h"
#include <math.h>
#include "kalmanfilter2.h"

// State being tracked
static float z_;  // position
static float v_;  // velocity

// 2x2 State Covariance matrix
static float Pzz_;
static float Pzv_;
static float Pvz_;
static float Pvv_;

static float zAccelVariance_;  // dynamic acceleration variance
static float zVariance_; //  z measurement noise variance fixed
	
void kalmanFilter2_configure(float zVariance, float zAccelVariance, float zInitial, float vInitial) {
	zAccelVariance_ = zAccelVariance;
	zVariance_ = zVariance;
	z_ = zInitial;
	v_ = vInitial;
	Pzz_ = 1500.0f;
	Pzv_ = 0.0f;
	Pvz_ = Pzv_;
	Pvv_ = 1500.0f;
	}


// Updates state given a sensor measurement of z, the acceleration noise variance,
// and the time in seconds since the last measurement.

void kalmanFilter2_predict(float zAccelVariance, float dt) {
	zAccelVariance_ = zAccelVariance;

	// predict state estimate.
	z_ += v_ * dt;

	// predict state covariance. The last term mixes in acceleration noise.
	Pzz_ += dt*Pzv_ + dt*Pvz_ + dt*dt*Pvv_ + zAccelVariance_*dt*dt*dt*dt/4.0f;
	Pzv_ +=                        dt*Pvv_ + zAccelVariance_*dt*dt*dt/2.0f;
	Pvz_ = Pzv_;
	Pvv_ +=                                + zAccelVariance_*dt*dt;
    }

void kalmanFilter2_update(float z, float* pZ, float* pV){
	// Update step.
	float y = z - z_;  // Innovation.
	float sInv = 1.0f / (Pzz_ + zVariance_);  // Innovation precision.
	float kz = Pzz_ * sInv;  // Kalman gain
	float kv = Pzv_ * sInv;

	// Update state estimate.
	z_ += kz * y;
	v_ += kv * y;

	*pZ = z_;
	*pV = v_;

	// Update state covariance.
	Pvv_ -= Pzv_ * kv;
	Pzv_ -= Pzv_ * kz;
	Pvz_  = Pzv_;
	Pzz_ -= Pzz_ * kz;

#if LOG_FILTER
	printf("%.1f %.1f %.1f %.1f %.1f\n", z, z_, Pzz_, v_, Pvv_);
#endif	
	}

