#ifndef CONFIG_H_
#define CONFIG_H_

// choose one of the following tests
// print the kalman filter P convergence over first 512 samples of data
#define LOG_CONVERGENCE    0
// print the baro sensor derived altitude cm, and kalman filter estimates for z and v, for all samples
#define LOG_INPUT_OUTPUT   1

// choose one of the following algorithms  
// for LOG_CONVERGENCE run, only KF3 and KF4 are valid
#define USE_KF2 1
#define USE_KF3 0
#define USE_KF4 0

#if (LOG_CONVERGENCE == 1)
	#define LOG_KF3_CONVERGENCE 1
	#define LOG_KF4_CONVERGENCE 1
#elif (LOG_INPUT_OUTPUT == 1) 
	#define LOG_KF3_CONVERGENCE 0
	#define LOG_KF4_CONVERGENCE 0
#endif

#define KF4_USE_DYNAMIC_ACCEL_BIAS_VARIANCE 1

// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f


#endif
