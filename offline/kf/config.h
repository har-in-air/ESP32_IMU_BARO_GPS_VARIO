#ifndef CONFIG_H_
#define CONFIG_H_

#define true 	1
#define false 	0

#define LOG_FILTER   true

// common static configuration options
#define KF_ZMEAS_VARIANCE       	200.0f
#define KF_ACCEL_VARIANCE			90.0f
#define KF_ACCEL_UPDATE_VARIANCE	50.0f
#define KF_ACCELBIAS_VARIANCE   	0.005f
#define KF_ADAPTIVE_ACCEL_FACTOR  	0.5f
#define IMU_SAMPLE_PERIOD_SECS		0.002f // IMU sensor samples @ 500Hz
#define KF_SAMPLE_PERIOD_SECS		0.02f // Kalman Filter output @ 50Hz

#endif
