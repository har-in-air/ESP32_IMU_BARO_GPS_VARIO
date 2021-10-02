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


#if LOG_KF3_CONVERGENCE

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
void kalmanFilter3_predict(float a, float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = a - State.b; // true acceleration = acceleration minus acceleration sensor bias
	State.z = State.z + (State.v * dt);
	State.v = State.v + (accel_true * dt);

	// Predict (a priori) State Covariance estimate P_k- = (F * P_k-1+ * F_t) + Q
	float dt2 = dt*dt;
	float dt3 = dt2*dt;
	float dt2div2 = dt2*0.5f;
	float dt3div2 = dt3*0.5f;
	float dt4div4 = dt2div2*dt2div2;
	
	float p00 = Pzz + 2.0f*dt*Pzv + dt2*(Pvv - Pzb) - dt3*Pvb + dt4div4*Pbb;
    float p01 = Pzv + dt*(Pvv - Pzb) - 3.0f*dt2div2*Pvb + dt3div2*Pbb;
	float p02 = Pzb + dt*Pvb - dt2div2*Pbb; 
	
	float p11 = Pvv - 2.0f*dt*Pvb +dt2*Pbb;
	float p12 = Pvb - dt*Pbb;

	Pzz = p00;
	Pzv = p01;
	Pzb = p02;
	
	Pvv = p11;
	Pvb = p12;
	
	//Pbb = Pbb; // no transformation

	//  add Q_k
    Pzz = Pzz + (dt4div4 * AccelVariance);
    Pzv = Pzv + (dt3div2 * AccelVariance);

    Pvv = Pvv + (dt2 * AccelVariance);

    Pbb = Pbb + BiasVariance;

	// P is symmetric
    Pvz = Pzv;
	Pbz = Pzb;
	Pbv = Pvb;
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

	// Update (a posteriori) state x_k+ = x_k- + (K_k*innov 
	State.z = State.z + kz * innov;
	State.v = State.v + kv * innov;
	State.b = State.b + kb * innov;
	
	// Update (a posteriori ) state covariance P_k+ = (I - K_k*H_k)*P_k-
	float p00 = Pzz - kz * Pzz;
	float p01 = Pzv - kz * Pzv;
	float p02 = Pzb - kz * Pzb;
	float p11 = Pvv - kv*Pzv;
	float p12 = Pvb - kv*Pzb;
	float p22 = Pbb - kb*Pzb;

	Pzz = p00;
	Pzv = p01;
	Pzb = p02;

	Pvv = p11;
	Pvb = p12;

    Pbb = p22;

	// P is symmetric
	Pvz = Pzv;
	Pbz = Pzb;
	Pbv = Pvb;

	// Return state variables of interest
	*pz = State.z;
	*pv = State.v;
	}
