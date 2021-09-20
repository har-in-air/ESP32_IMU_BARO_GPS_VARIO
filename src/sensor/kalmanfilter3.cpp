#include "common.h"
#include <math.h>
#include "config.h"
#include "kalmanfilter3.h"

static const char* TAG = "kalmanfilter3";

typedef struct KF3_STATE_ {
	float z; // altitude
	float v; // climb/sink rate
	float b; // acceleration residual bias (post-calibration)
} KF3_STATE;

static KF3_STATE  State;


#if LOG_KF3

#define STABLE_COUNT_THRESHOLD 0

typedef struct KF3_LOG_ {
	float z; // altitude
	float v; // climb/sink rate
	float b; // acceleration residual bias (post-calibration)
	float pzz; // altitude covariance
	float pvv; // climb/sink rate covariance
	float pbb; // acceleration residual bias (post-calibration) covariance
} KF3_LOG;

static int  StableCounter = 0;
static bool LogEnabled = true;
static int  SampleIndex = 0;
static KF3_LOG Log[NUM_TEST_SAMPLES];
#endif


// 3x3 Process model state Covariance matrix
// Pzz Pzv Pzb
// Pvz Pvv Pvb
// Pbz Pbv Pbb
// Note: the covariance matrix is symmetric

static float Pzz;
static float Pzv;
static float Pzb;

static float Pvz;
static float Pvv;
static float Pvb;

static float Pbz;
static float Pbv;
static float Pbb;

static float AccelVariance; // environmental acceleration variance, depends on conditions
static float BiasVariance; // assume a low value for acceleration bias noise variance

// measurement model sensor noise variance, measured offline
static float ZSensorVariance; //  altitude measurement noise variance

// Tracks the position z and velocity v of an object moving in a straight line,
// (here assumed to be vertical) that is perturbed by random accelerations.
// sensor measurement of z is assumed to have constant measurement noise 
// variance zSensorVariance. This can be calculated off-line for the specific sensor.
// AccelVariance will depend on the conditions (more for highly thermic/turbulent conditions)
// BiasVariance can be set low as it is not expected to change much
// zInitial can be determined by averaging a few samples of the altitude measurement.
// vInitial can be set as 0

void kalmanFilter3_configure(float zSensorVariance, float aVariance, float bVariance, float zInitial, float vInitial){
	ZSensorVariance = zSensorVariance;
	AccelVariance = aVariance;
    BiasVariance = bVariance;

	State.z = zInitial;
	State.v = vInitial;
	State.b = 0.0f; // let residual acceleration bias = 0 initially

	Pzz = 400.0f;
	Pzv = 0.0f;
	Pzb = 0.0f;
	
	Pvz = Pzv;
	Pvv = 400.0f;
	Pvb = 0.0f;
	
	Pbz = Pzb;
	Pbv = Pvb;
	Pbb = 400.0f;
	}


// gravity-compensated earth-z accel in cm/s/s,  and elapsed time dt in seconds
void kalmanFilter3_predict(float am, float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = am - State.b; // true acceleration = measured acceleration minus acceleration sensor bias
	State.z = State.z + (State.v * dt);
	State.v = State.v + (accel_true * dt);

	// Predict State Covariance matrix
	float t00,t01,t02;
	float t10,t11,t12;
	float t20,t21,t22;
	
	float dt2div2 = dt*dt*0.5f;
	float dt3div2 = dt2div2*dt;
	float dt4div4 = dt2div2*dt2div2;
	
	t00 = Pzz + dt*Pvz - dt2div2*Pbz;
	t01 = Pzv + dt*Pvv - dt2div2*Pbv;
	t02 = Pzb + dt*Pvb - dt2div2*Pbb;

	t10 = Pvz - dt*Pbz;
	t11 = Pvv - dt*Pbv;
	t12 = Pvb - dt*Pbb;

	t20 = Pbz;
	t21 = Pbv;
	t22 = Pbb;
	
	Pzz = t00 + dt*t01 - dt2div2*t02;
	Pzv = t01 - dt*t02;
	Pzb = t02;
	
	Pvz = Pzv;
	Pvv = t11 - dt*t12;
	Pvb = t12;
	
	Pbz = Pzb;
	Pbv = Pvb;
	Pbb = t22;

	//  add Q_k
    Pzz += dt4div4*AccelVariance;
    Pzv += dt3div2*AccelVariance;

    Pvz += dt3div2*AccelVariance;
    Pvv += dt*dt*AccelVariance;

    Pbb += BiasVariance;
	}


// Update state and state covariance matrix P given a sensor z measurement 
void kalmanFilter3_update(float zm, float* pz, float* pv) {
	// Error
	float innov = zm - State.z; 
	float sInv = 1.0f / (Pzz + ZSensorVariance);  

    // Kalman gains
	float kz = Pzz * sInv;  
	float kv = Pvz * sInv;
	float kb = Pbz * sInv;

	// Update state 
	State.z = State.z + kz * innov;
	State.v = State.v + kv * innov;
	State.b = State.b + kb * innov;
	
	// Update state covariance matrix
	Pbz -= kb * Pzz;
	Pbv -= kb * Pzv;
	Pbb -= kb * Pzb;
	
	Pvz -= kv * Pzz;
	Pvv -= kv * Pzv;
	Pvb -= kv * Pzb;
	
	Pzz -= kz * Pzz;
	Pzv -= kz * Pzv;
	Pzb -= kz * Pzb;

	*pz = State.z;
	*pv = State.v;

#if LOG_KF3
	StableCounter++;
	if ((StableCounter > STABLE_COUNT_THRESHOLD) && (LogEnabled == true)) {
		Log[SampleIndex].z = State.z;
		Log[SampleIndex].v = State.v;
		Log[SampleIndex].b = State.b;
		Log[SampleIndex].pzz = Pzz;
		Log[SampleIndex].pvv = Pvv;
		Log[SampleIndex].pbb = Pbb;
		SampleIndex++;
		if (SampleIndex >= NUM_TEST_SAMPLES) {
			LogEnabled = false;
			printf("KF3 log\n");
			for (int inx = 0; inx < NUM_TEST_SAMPLES; inx++) {
				printf("z %d (%.1f), v %.1f (%.1f), b %.1f (%.1f)\n", (int)(Log[inx].z+0.5f),Log[inx].pzz, Log[inx].v, Log[inx].pvv, Log[inx].b, Log[inx].pbb);
				}
			}
		}
#endif

	}
