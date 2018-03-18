//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
// modified HN to use acceleration feedback based on input flag
#ifndef IMU_H_
#define IMU_H_

extern float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame


void imu_mahonyAHRSupdate9DOF(int bUseAccel, int bUseMag, float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void imu_mahonyAHRSupdate6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az);
void imu_madgwickAHRSupdate6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az);

// HN
void imu_quaternion2YawPitchRoll(float q0, float q1, float q2, float q3, float* pYawDeg, float* pPitchDeg, float* pRollDeg);
float imu_gravityCompensatedAccel(float ax, float ay, float az, float q0, float q1, float q2, float q3);
#endif
