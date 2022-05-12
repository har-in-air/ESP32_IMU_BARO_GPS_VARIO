#ifndef CONFIG_H_
#define CONFIG_H_

#define true 	1
#define false 	0

#define LOG_FILTER   true

// Kalman filter configuration
#define KF_ACCEL_VARIANCE     100

// adaptive uncertainty injection
#define KF_ADAPT_DEFAULT     100


#define KF_ACCELBIAS_VARIANCE   	0.005f

// KF4 Acceleration Measurement Noise variance
#define KF_A_MEAS_VARIANCE_4		10000.0f
#define KF_A_MEAS_VARIANCE_4D  		10.0f

// KF4 Altitude Measurement Noise Variance
#define KF_Z_MEAS_VARIANCE			200.0f


// common static configuration options
#define IMU_SAMPLE_PERIOD_SECS		0.002f // IMU sensor samples @ 500Hz
#define KF_SAMPLE_PERIOD_SECS		0.02f // Kalman Filter output @ 50Hz

#endif
