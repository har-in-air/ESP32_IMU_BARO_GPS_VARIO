#include "common.h"
#include <math.h>
#include "config.h"
#include "kalmanfilter4.h"

static const char* TAG = "kalmanfilter4";

typedef struct KF4_STATE_ {
	float z; // altitude
	float v; // climb/sink rate
	float a; // gravity-compensated net earth-z axis acceleration
	float b; // acceleration residual bias (post-calibration)
} KF4_STATE;

static KF4_STATE  State;

// 4x4 process model state covariance estimate P 
// Note : P is symmetric
// Pzz Pzv Pza Pzb
// Pvz Pvv Pva Pvb
// Paz Pav Paa Pab
// Pbz Pbv Pba Pbb

static float Pzz;
static float Pzv;
static float Pza;
static float Pzb;

static float Pvz; 
static float Pvv;
static float Pva;
static float Pvb;

static float Paz; 
static float Pav; 
static float Paa;
static float Pab;

static float Pbz; 
static float Pbv; 
static float Pba; 
static float Pbb;

// 4x4 Process model noise covariance  Q
// | 0  0  0     0     |
// | 0  0  0     0     |
// | 0  0  a_var 0     |
// | 0  0  0     b_var |
static float AccelVariance; // environmental acceleration variance, depends on conditions
static float BiasVariance; // assume a low value for acceleration bias noise variance

// measurement model sensor noise variance, measured offline
static float ZSensorVariance; //  altitude measurement noise variance
static float AUpdateVariance; //  acceleration update noise variance

// dynamically update the additive uncertainty (noise) added in the update
// phase to the Pvv and Pbb elements of the state covariance matrix
static bool UseAdaptiveVariance = false;

// Tracks the position z, and velocity v of an object moving in a straight line,
// that is perturbed by random accelerations. Assumes we have sensors providing
// periodic measurements for acceleration a and position z. 
// zSensorVariance can be calculated offline with the unit at rest.
// zInitial can be determined by averaging a few samples of the altitude measurement.
// vInitial and aInitial can be set to zero.
// aVariance should be specified with a large enough value to allow the true state (z, v) to be within the uncertainty estimate.
// It should be set higher for more thermic/turbulent conditions

void kalmanFilter4_configure(float zSensorVariance, float aVariance, bool bAdaptUpdateVariance, float zInitial, float vInitial, float aInitial){
	ZSensorVariance = zSensorVariance;
	AUpdateVariance = KF_ACCEL_UPDATE_VARIANCE*1000.0f;
	AccelVariance = aVariance;
    BiasVariance = KF_ACCELBIAS_VARIANCE;
	UseAdaptiveVariance = bAdaptUpdateVariance;

	State.z = zInitial;
	State.v = vInitial;
    State.a = aInitial;
	State.b = 0.0f; // assume residual acceleration bias = 0 initially

	Pzz = 1500.0f;
    Pzv = 0.0f;
	Pza = 0.0f;
	Pzb = 0.0f;
	
	Pvz = Pzv; 
	Pvv = 1500.0f;
	Pva = 0.0f;
	Pvb = 0.0f;
	
	Paz = Pza;
	Pav = Pva;
	Paa = 50000.0f;
	Pab = 0.0f;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = 1500.0f;
	}


// Process model state transition matrix F  (4x4)
//  | 1   dt  dt^2/2 -dt^2/2 |
//  | 0   1   dt     -dt     |
//  | 0   0   1       0      |
//  | 0   0   0       1      |
//
void kalmanFilter4_predict(float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = State.a - State.b; // true acceleration = acceleration minus acceleration sensor bias
	State.z = State.z + (State.v * dt) + (accel_true * dt * dt* 0.5f);
	State.v = State.v + (accel_true * dt);

	// Predicted (a priori) state covariance estimate P_k- = (F * P_k-1+ * F_t) + Qk
	float dt2 = dt*dt;  // dt^2
	float dt3 = dt2*dt; // dt^3 
	float dt4 = dt2*dt2; // dt^4;
	float dt2div2 = dt2*0.5f; // dt^2/2
	float dt3div2 = dt3*0.5f; // dt^3/2
	float dt4div2 = dt4*0.5f; // dt^4/2
	float dt4div4 = dt4*0.25f; // dt^4/4
	
	float p00 = Pzz + 2.0f*Pzv*dt + (Pza - Pzb)*dt2  + Pvv*dt2div2 + (Pva - Pvb)*dt3 + (Paa+Pbb)*dt4div4 - Pab*dt4div2;
	float p01 = Pzv + dt*(Pza - Pzb + Pvv) + 3.0f*dt2div2*(Pva - Pvb) - Pab*dt3 + (Paa + Pbb)*dt3div2;
	float p02 = Pza + Pva*dt + (Paa - Pba)*dt2div2;
	float p03 = Pzb + Pvb*dt + (Pab - Pbb)*dt2div2;

	float p11 = Pvv + 2.0f*dt*(Pva - Pvb) + dt2*(Paa - 2.0f*Pab + Pbb);
	float p12 = Pva + dt*(Paa - Pba);
	float p13 = Pvb + dt*(Pab - Pbb);

	float p22 = Paa;
	float p23 = Pab;
	float p33 = Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// Add Q_k
	Paa = Paa + AccelVariance;
	Pbb = Pbb + BiasVariance;
	}


// Update state estimate and state covariance estimate, given new z and a measurements
// Measurement vector m_k
// m_k = | zm_k |
//       | am_k |
// H matrix transforms state vector space to measurement space 
// | 1 0 0 0 |
// | 0 0 1 0 |
// Predicted measurement from a_priori state estimate = (H * x_k-)
// Innovation error y_k = actual measurement m_k minus predicted measurement
// y_k = m_k - (H * x_k-)
// 2x2 sensor noise covariance matrix R_k
// | zsensor_variance  0                |  
// | 0                 asensor_variance |

void kalmanFilter4_update(float zm, float am, float* pz, float* pv) {
	// Innovation Error y_k = measurement minus apriori estimate
	float z_err = zm - State.z;
	float a_err = am - State.a;

	// Innovation covariance S_k
	// S_k = (H * P_k- * H_t) + R_k
	float s00 = Pzz;
	float s01 = Pza;
	float s10 = s01;
	float s11 = Paa;

	// add R_k
	s00 = s00 + ZSensorVariance;
	if (UseAdaptiveVariance) {	
		float accel_ext = (am-State.b)*(am-State.b);
		// inject additional uncertainty depending on the magnitude of the external acceleration.
		// allows filter  to respond quickly to moderate/large accelerations while heavily filtering out noise
		// when there is low or no acceleration
		s11 = s11 + KF_ADAPTIVE_ACCEL_FACTOR*accel_ext;
		// allow system to update estimated bias only when there is low acceleration
		BiasVariance = 1.0f/(1.0f + 2.0f*accel_ext);	
		}
	else {
		s11 = s11 + AUpdateVariance;
		}

	// Compute S_k_inv
	float sdetinv = 1.0f/(s00*s11 - s10*s01);
	float sinv00 = sdetinv * s11;
	float sinv01 = -sdetinv * s10;
	float sinv10 = sinv01;
	float sinv11 = sdetinv * s00;

	// Kalman gain K_k [4x2] matrix
	// K_k = P_k- * H_t * S_k_inv
	float k00 = Pzz*sinv00 + Pza*sinv10;  
	float k01 = Pzz*sinv01 + Pza*sinv11;
	float k10 = Pvz*sinv00 + Pva*sinv10;
	float k11 = Pvz*sinv01 + Pva*sinv11;
	float k20 = Paz*sinv00 + Paa*sinv10;
	float k21 = Paz*sinv01 + Paa*sinv11;
	float k30 = Pbz*sinv00 + Pba*sinv10;
	float k31 = Pbz*sinv01 + Pba*sinv11;

	// Updated (a posteriori) state estimate x_k+ 
	// x_k+ = x_k- + K_k * y_k
	State.z = State.z + (k00*z_err + k01*a_err);
	State.v = State.v + (k10*z_err + k11*a_err);
	State.a = State.a + (k20*z_err + k21*a_err);
	State.b = State.b + (k30*z_err + k31*a_err);

	// Updated (a posteriori) state covariance estimate P_k+
	// P_k+ = (I - K_k * H_k)*P_k-
	float tmp = 1.0f - k00;
	float p00 = tmp*Pzz - k01*Paz;
	float p01 = tmp*Pzv - k01*Pav;
	float p02 = tmp*Pza - k01*Paa;
	float p03 = tmp*Pzb - k01*Pab;

	float p11 = -k10*Pzv + Pvv - k11*Pav;
	float p12 = -k10*Pza + Pva - k11*Paa;
	float p13 = -k10*Pzb + Pvb - k11*Pab;

	float p22 = -k20*Pza + (1.0f-k21)*Paa;
	float p23 = -k20*Pzb + (1.0f-k21)*Pab;

	float p33 = -k30*Pzb -k31*Pab + Pbb;

	Pzz = p00;
	Pzv = p01;
	Pza = p02;
	Pzb = p03;

	Pvz = Pzv;
	Pvv = p11;
	Pva = p12;
	Pvb = p13;

	Paz = Pza;
	Pav = Pva;
	Paa = p22;
	Pab = p23;

	Pbz = Pzb;
	Pbv = Pvb;
	Pba = Pab;
	Pbb = p33; 

	// return the state variables of interest (z and v)
	*pz = State.z;
	*pv = State.v;

#if LOG_FILTER
	// WARNING : this should only be enabled for offline analysis of a downloaded data log !
	// e.g. see test code in /offline/kf 
	printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",zm, State.z, Pzz, State.v, Pvv, State.a - State.b, Paa, State.b, Pbb);
#endif
	}
