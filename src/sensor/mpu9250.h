/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

modified HN
*/

#ifndef MPU9250_h
#define MPU9250_h


typedef enum GyroRange_
{
GYRO_RANGE_250DPS,
GYRO_RANGE_500DPS,
GYRO_RANGE_1000DPS,
GYRO_RANGE_2000DPS
} GyroRange;

typedef enum AccelRange_
{
ACCEL_RANGE_2G,
ACCEL_RANGE_4G,
ACCEL_RANGE_8G,
ACCEL_RANGE_16G    
} AccelRange;

typedef enum DlpfBandwidth_
{
DLPF_BANDWIDTH_184HZ,
DLPF_BANDWIDTH_92HZ,
DLPF_BANDWIDTH_41HZ,
DLPF_BANDWIDTH_20HZ,
DLPF_BANDWIDTH_10HZ,
DLPF_BANDWIDTH_5HZ
} DlpfBandwidth;

typedef enum LpAccelOdr_
{
LP_ACCEL_ODR_0_24HZ = 0,
LP_ACCEL_ODR_0_49HZ = 1,
LP_ACCEL_ODR_0_98HZ = 2,
LP_ACCEL_ODR_1_95HZ = 3,
LP_ACCEL_ODR_3_91HZ = 4,
LP_ACCEL_ODR_7_81HZ = 5,
LP_ACCEL_ODR_15_63HZ = 6,
LP_ACCEL_ODR_31_25HZ = 7,
LP_ACCEL_ODR_62_50HZ = 8,
LP_ACCEL_ODR_125HZ = 9,
LP_ACCEL_ODR_250HZ = 10,
LP_ACCEL_ODR_500HZ = 11
} LpAccelOdr;


int mpu9250_config();
int mpu9250_setSrd(uint8_t srd);
int mpu9250_enableDataReadyInterrupt();
int mpu9250_disableDataReadyInterrupt();


int mpu9250_calibrateGyro();
int mpu9250_calibrateAccel();
int mpu9250_calibrateMag();

int mpu9250_getGyroAccelMagData(float* pgx, float* pgy, float* pgz, float* pax, float* pay, float* paz, float* pmx, float* pmy, float* pmz);
void mpu9250_initCalibrationParams(void);
int mpu9250_getVector(uint8_t startAddr, int isLittleEndian, int16_t* px, int16_t* py, int16_t* pz);

#endif
