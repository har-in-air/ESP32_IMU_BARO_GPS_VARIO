#include "common.h"
#include <math.h>
#include "config.h"
#include "kalmanfilter3.h"

// State being tracked
static float z_;  // position
static float v_;  // velocity
static float aBias_;  // acceleration

// 3x3 State Covariance matrix
static float Pzz_;
static float Pzv_;
static float Pza_;
static float Pvz_;
static float Pvv_;
static float Pva_;
static float Paz_;
static float Pav_;
static float Paa_;

static float zAccelBiasVariance_; // assumed fixed.
static float zAccelVariance_;  // environmental acceleration variance, depends on conditions
static float zMeasVariance_; //  sensor z measurement noise variance, measure offline, fixed


static const char* TAG = "kalmanfilter3";

// Tracks the position z and velocity v of an object moving in a straight line,
// (here assumed to be vertical) that is perturbed by random accelerations.
// sensor measurement of z is assumed to have constant measurement noise 
// variance zVariance.
// This can be calculated offline for the specific sensor.
// zInitial can be determined by averaging a few samples of the altitude measurement.
// vInitial and aBiasInitial can be set as 0.0
// zAccelVariance can be specified with a large initial value to skew the 
// filter towards the fresh data.


void kalmanFilter3_configure(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial, float aBiasInitial) {
	zMeasVariance_ = zMeasVariance;
	zAccelVariance_ = zAccelVariance;
    zAccelBiasVariance_ = zAccelBiasVariance;
#ifdef KF_DEBUG
	ESP_LOGI(TAG, "zMeasVariance %d\r\nzAccelVariance %d", (int)zMeasVariance, (int)zAccelVariance);
#endif	

	z_ = zInitial;
	v_ = vInitial;
	aBias_ = aBiasInitial;
	Pzz_ = 100.0f;
	Pzv_ = 0.0f;
	Pza_ = 0.0f;
	
	Pvz_ = 0.0f;
	Pvv_ = 100.0f;
	Pva_ = 0.0f;
	
	Paz_ = 0.0f;
	Pav_ = 0.0;
	Paa_ = 100000.0f;
	}


// predict state [Z, V] and state covariance matrix given z acceleration 
// input a in cm/s^2,  and elapsed time dt in seconds
void kalmanFilter3_predict(float a, float dt) {
	// Predict state
	float accel = a - aBias_;
	v_ += accel * dt;
	z_ += v_ * dt;

	// Predict State Covariance matrix
	float t00,t01,t02;
	float t10,t11,t12;
	float t20,t21,t22;
	
	float dt2div2 = dt*dt/2.0f;
	float dt3div2 = dt2div2*dt;
	float dt4div4 = dt2div2*dt2div2;
	
	t00 = Pzz_ + dt*Pvz_ - dt2div2*Paz_;
	t01 = Pzv_ + dt*Pvv_ - dt2div2*Pav_;
	t02 = Pza_ + dt*Pva_ - dt2div2*Paa_;

	t10 = Pvz_ - dt*Paz_;
	t11 = Pvv_ - dt*Pav_;
	t12 = Pva_ - dt*Paa_;

	t20 = Paz_;
	t21 = Pav_;
	t22 = Paa_;
	
	Pzz_ = t00 + dt*t01 - dt2div2*t02;
	Pzv_ = t01 - dt*t02;
	Pza_ = t02;
	
	Pvz_ = t10 + dt*t11 - dt2div2*t12;
	Pvv_ = t11 - dt*t12;
	Pva_ = t12;
	
	Paz_ = t20 + dt*t21 - dt2div2*t22;
	Pav_ = t21 - dt*t22;
	Paa_ = t22;

    Pzz_ += dt4div4*zAccelVariance_;
    Pzv_ += dt3div2*zAccelVariance_;

    Pvz_ += dt3div2*zAccelVariance_;
    Pvv_ += dt*dt*zAccelVariance_;

    Paa_ += zAccelBiasVariance_;
	}


// Updates state [Z, V] and state covariance matrix P given a sensor z measurement 
void kalmanFilter3_update(float z, float* pZ, float* pV) {
	// Error
	float innov = z - z_; 
	float sInv = 1.0f / (Pzz_ + zMeasVariance_);  

    // Kalman gains
	float kz = Pzz_ * sInv;  
	float kv = Pvz_ * sInv;
	float ka = Paz_ * sInv;

	// Update state 
	z_ += kz * innov;
	v_ += kv * innov;
	aBias_ += ka * innov;
	
	*pZ = z_;
	*pV = v_;

	// Update state covariance matrix
	Paz_ -= ka * Pzz_;
	Pav_ -= ka * Pzv_;
	Paa_ -= ka * Pza_;
	
	Pvz_ -= kv * Pzz_;
	Pvv_ -= kv * Pzv_;
	Pva_ -= kv * Pza_;
	
	Pzz_ -= kz * Pzz_;
	Pzv_ -= kz * Pzv_;
	Pza_ -= kz * Pza_;
	}
