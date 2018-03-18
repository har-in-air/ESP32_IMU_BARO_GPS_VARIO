/*
MPU9250.cpp
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

modified by HN (used register declarations and configuration code)
*/

#include "common.h"
#include "config.h"
#include "vspi.h"
#include "options.h"
#include "calib.h"
#include "mpu9250.h"
#include "lcd7565.h"
#include "cct.h"

static int16_t axBias_;
static int16_t ayBias_;
static int16_t azBias_;
static int16_t gxBias_;
static int16_t gyBias_;
static int16_t gzBias_;
static int16_t mxBias_;
static int16_t myBias_;
static int16_t mzBias_;
static int16_t mxSens_;
static int16_t mySens_;
static int16_t mzSens_;
static float gyroScale_;
static float accelScale_;
static float magXScale_;
static float magYScale_;
static float magZScale_;

const uint8_t SPI_READ = 0x80;

const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_184 = 0x01;
const uint8_t ACCEL_DLPF_92 = 0x02;
const uint8_t ACCEL_DLPF_41 = 0x03;
const uint8_t ACCEL_DLPF_20 = 0x04;
const uint8_t ACCEL_DLPF_10 = 0x05;
const uint8_t ACCEL_DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_184 = 0x01;
const uint8_t GYRO_DLPF_92 = 0x02;
const uint8_t GYRO_DLPF_41 = 0x03;
const uint8_t GYRO_DLPF_20 = 0x04;
const uint8_t GYRO_DLPF_10 = 0x05;
const uint8_t GYRO_DLPF_5 = 0x06;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;
// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03; 
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;


static int mpu9250_writeRegister(uint8_t subAddress, uint8_t data);
static int mpu9250_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
static int mpu9250_writeAK8963Register(uint8_t subAddress, uint8_t data);
static int mpu9250_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
static int mpu9250_whoAmI();
static int mpu9250_whoAmIAK8963();

static int asaX_, asaY_, asaZ_;

#define TAG "mpu9250"

extern volatile int DrdyFlag;

int mpu9250_config(){   
   // select clock source to gyro
   if(mpu9250_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
      return -1;
      }
   // enable I2C master mode
   if(mpu9250_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
      return -2;
      }
   // set the I2C bus speed to 400 kHz
   if(mpu9250_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
      return -3;
      }
   // set AK8963 to Power Down
   mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
   // reset the MPU9250
   mpu9250_writeRegister(PWR_MGMNT_1,PWR_RESET);
   // wait for MPU-9250 to come back up
   delayMs(1);
   // reset the AK8963
   mpu9250_writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
   // select clock source to gyro
   if(mpu9250_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
      return -4;
      }
   // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
   if ((mpu9250_whoAmI() != 113)&&(mpu9250_whoAmI() != 115)){
      ESP_LOGE(TAG, "whoami error");
      return -5;
      }
   // enable accelerometer and gyro
   if(mpu9250_writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
      return -6;
      }

   if(mpu9250_writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
      return -7;
      }
   accelScale_ = 4000.0f/32767.5f;

   if(mpu9250_writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
      return -8;
      }
   gyroScale_ = 1000.0f/32767.5f;

   if(mpu9250_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ 
      return -9;
      } 
   if(mpu9250_writeRegister(CONFIG,GYRO_DLPF_184) < 0){
      return -10;
      }
   // setting the sample rate divider to 0 as default
   if(mpu9250_writeRegister(SMPDIV,0x00) < 0){ 
      return -11;
      } 
   //_srd = 0;
   // enable I2C master mode
   if(mpu9250_writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	   return -12;
      }
	// set the I2C bus speed to 400 kHz
	if( mpu9250_writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	   }
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( mpu9250_whoAmIAK8963() != 72 ){
      return -14;
	   }
   // get the magnetometer calibration
   // set AK8963 to Power Down
   if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -15;
      }
   delayMs(100); // long wait between AK8963 mode changes
   // set AK8963 to FUSE ROM access
   if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
      return -16;
      }
   delayMs(100); // long wait between AK8963 mode changes
   // read the AK8963 ASA registers and compute magnetometer scale factors
   uint8_t buffer[3];
   mpu9250_readAK8963Registers(AK8963_ASA, 3 , buffer);
   asaX_ = (int)buffer[0];
   asaY_ = (int)buffer[1];
   asaZ_ = (int)buffer[2];
   ESP_LOGI(TAG,"asaX %d asaY %d asaZ %d", asaX_, asaY_, asaZ_);

   // set AK8963 to Power Down
   if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -17;
      }
   delayMs(100); // long wait between AK8963 mode changes  
   // set AK8963 to 16 bit resolution, 100 Hz update rate
   if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
      return -18;
      }
   delayMs(100); // long wait between AK8963 mode changes
   // select clock source to gyro
   if(mpu9250_writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
      return -19;
      }       
   // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
   mpu9250_readAK8963Registers(AK8963_HXL,7,buffer);

   mpu9250_setSrd(1); // accel+gyro odr = 500Hz, mag odr = 100Hz

   if (mpu9250_writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
      return -21;
      }  
   if (mpu9250_writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
      return -22;
      }

   mpu9250_initCalibrationParams();
   return 1;
   }



int mpu9250_setSrd(uint8_t srd) {
   uint8_t buffer[7];
   // setting the sample rate divider to 19 to facilitate setting up magnetometer
   if(mpu9250_writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
      return -1;
      }   
   if(srd > 9){
      // set AK8963 to Power Down
      if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
         return -2;
         }
      delayMs(100); // long wait between AK8963 mode changes  
      // set AK8963 to 16 bit resolution, 8 Hz update rate
      if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
         return -3;
         }
      delayMs(100); // long wait between AK8963 mode changes     
      // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
      mpu9250_readAK8963Registers(AK8963_HXL,7,buffer);
      } 
   else {
      // set AK8963 to Power Down
      if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
         return -2;
         }
      delayMs(100); // long wait between AK8963 mode changes  
      // set AK8963 to 16 bit resolution, 100 Hz update rate
      if(mpu9250_writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
         return -3;
         }
      delayMs(100); // long wait between AK8963 mode changes     
      // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
      mpu9250_readAK8963Registers(AK8963_HXL,7,buffer);    
      } 
   // setting the sample rate divider 
   if(mpu9250_writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
      return -4;
      } 
   //_srd = srd;
   return 1; 
   }


void mpu9250_initCalibrationParams(void) {
#if 0
   // valid for +/-4G FS
axBias_ = -100;
ayBias_ = -158;
azBias_ = -359;
#endif
	accelScale_ = 4000.0f/32767.5f; // accel in milli-gs
#if 0    
   // valid for FS=1000dps
gxBias_ = -88;
gyBias_ = 138;
gzBias_ = -10;

#endif	
   gyroScale_ = 1000.0f/32767.5f;  // rotation rate in deg/second
	
#if 0	 
 mxBias_ = 278;
 myBias_ = -67;
 mzBias_ = -19;
 mxSens_ = 283;
 mySens_ = 265;
 mzSens_ = 285;

#endif
	axBias_ = calib.axBias;
	ayBias_ = calib.ayBias;
	azBias_ = calib.azBias;
	gxBias_ = calib.gxBias;
	gyBias_ = calib.gyBias;
	gzBias_ = calib.gzBias;
	mxBias_ = calib.mxBias;
	myBias_ = calib.myBias;
	mzBias_ = calib.mzBias;
	mxSens_ = calib.mxSens;
	mySens_ = calib.mySens;
	mzSens_ = calib.mzSens;
#ifdef MPU9250_DEBUG
	ESP_LOGI(TAG,"--- Calibration parameters from file ---");
	ESP_LOGI(TAG,"Accel  : axBias %d, ayBias %d, azBias %d",axBias_, ayBias_, azBias_);
	ESP_LOGI(TAG,"Gyro   : gxBias %d, gyBias %d, gzBias %d",gxBias_, gyBias_, gzBias_);
	ESP_LOGI(TAG,"Mag    : mxBias %d, myBias %d, mzBias %d",mxBias_, myBias_, mzBias_);
	ESP_LOGI(TAG,"Mag    : mxSens %d, mySens %d, mzSens %d",mxSens_, mySens_, mzSens_);
#endif
	
	magXScale_ = mxSens_ ? 1000.0f/(float)mxSens_ : 1.0f;
	magYScale_ = mySens_ ? 1000.0f/(float)mySens_ : 1.0f;
	magZScale_ = mzSens_ ? 1000.0f/(float)mzSens_ : 1.0f;
	}



int mpu9250_getGyroAccelMagData(float* pgx, float* pgy, float* pgz, float* pax, float* pay, float* paz,float* pmx, float* pmy, float* pmz) {
   uint8_t buffer[21];
   if (mpu9250_readRegisters(ACCEL_OUT, 21, buffer) < 0) {
      return -1;
      }
   int16_t x, y, z;
   x = (((int16_t)buffer[0]) << 8) | buffer[1];  
   y = (((int16_t)buffer[2]) << 8) | buffer[3];
   z = (((int16_t)buffer[4]) << 8) | buffer[5];
   *pax = accelScale_*(float)(x - axBias_);
   *pay = accelScale_*(float)(y - ayBias_);
   *paz = accelScale_*(float)(z - azBias_);

   x = (((int16_t)buffer[8]) << 8) | buffer[9];
   y = (((int16_t)buffer[10]) << 8) | buffer[11];
   z = (((int16_t)buffer[12]) << 8) | buffer[13];
   *pgx = gyroScale_*(float)(x - gxBias_);
   *pgy = gyroScale_*(float)(y - gyBias_);
   *pgz = gyroScale_*(float)(z - gzBias_);

   // note mag data is little endian
   x = (((int16_t)buffer[15]) << 8) | buffer[14];
   y = (((int16_t)buffer[17]) << 8) | buffer[16];
   z = (((int16_t)buffer[19]) << 8) | buffer[18];
   int xi = ((int)x * (asaX_+128))/256;
   int yi = ((int)y * (asaY_+128))/256;
   int zi = ((int)z * (asaZ_+128))/256;

   *pmx = magXScale_*(float)(xi - mxBias_);
   *pmy = magYScale_*(float)(yi - myBias_);
   *pmz = magZScale_*(float)(zi - mzBias_);
   return 1;
   }


// place unit so that the sensor board accelerometer z axis points 
// vertically (up or down). This is where the sensor z axis sees a static 
// acceleration of 1g (or -1g). In this orientation the ax and ay values are 
// the offsets (biases) for a 0g environment. 
// Repeat this calibration a few times with the debug serial monitor to check the 
// consistency of the calibration offsets. The board MUST be in a 1g static acceleration 
// environment for this calibration, i.e. at rest, no vibrations etc.

#define ACCEL_NUM_AVG_SAMPLES	50

int mpu9250_calibrateAccel(void){
	int16_t ax,ay,az,az1g;
	int32_t axAccum, ayAccum, azAccum;
   ESP_LOGI(TAG, "Calibrating accelerometer");
	axAccum = ayAccum = azAccum = 0;
   if(mpu9250_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ 
      ESP_LOGE(TAG,"accel calib : error reducing bandwidth to 20Hz");
      return -1;
      } 
   delayMs(500);
	for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
      taskYIELD();
      cct_delayUs(2000); 
      if (mpu9250_getVector(ACCEL_OUT,false, &ax, &ay, &az) < 0) {
         ESP_LOGE(TAG, "accel calib : error reading accel data");
         return -2;
         }
		axAccum += (int32_t) ax;
		ayAccum += (int32_t) ay;
		azAccum += (int32_t) az;
		}
	axBias_ = (int16_t)(axAccum / ACCEL_NUM_AVG_SAMPLES);
	ayBias_ = (int16_t)(ayAccum / ACCEL_NUM_AVG_SAMPLES);
	az1g = (int16_t)(azAccum / ACCEL_NUM_AVG_SAMPLES);

   azBias_ = az1g > 0 ? az1g - (int16_t)(1000.0f/accelScale_) : az1g + (int16_t)(1000.0f/accelScale_);
   ESP_LOGI(TAG, "axBias = %d\r\nayBias = %d\r\nazBias = %d", (int)axBias_, (int)ayBias_, (int)azBias_);
   calib.axBias = axBias_;
   calib.ayBias = ayBias_;
   calib.azBias = azBias_;
   calib_save(); // update calibration file with accel calibration parameter

   if(mpu9250_writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ 
      ESP_LOGE(TAG,"accel calib : error resetting bandwidth to 184Hz");
      return -3;
      } 
   return 1;
	}
	



#define GYRO_NUM_CALIB_SAMPLES			50
	
int mpu9250_calibrateGyro(void) {
	int16_t gx,gy,gz;
	int32_t gxAccum, gyAccum, gzAccum;
	int foundBadData;
	int numTries = 1;
   ESP_LOGI(TAG, "Calibrating gyro");
   // reduce bandwidth to reduce noise power
   if(mpu9250_writeRegister(CONFIG,GYRO_DLPF_20) < 0){
      ESP_LOGE(TAG,"gyro calib : error reducing bandwidth to 20Hz");
      return -1;
      }

	do {
		delayMs(500);
		foundBadData = 0;
		gxAccum = gyAccum = gzAccum = 0;
		for (int inx = 0; inx < GYRO_NUM_CALIB_SAMPLES; inx++){
         taskYIELD();
         cct_delayUs(2000); 
			if (mpu9250_getVector(GYRO_OUT,false, &gx, &gy, &gz) < 0) {
            ESP_LOGE(TAG, "gyro calib : error reading data");
            return -2;
            }
         int16_t maxOffset = opt.misc.gyroOffsetLimit1000DPS;
			//ESP_LOGI(TAG, "[%d] %d %d %d", inx, gx, gy, gz);
			// if a larger than expected gyro bias is measured, assume the unit was disturbed and try again
         // after a short delay, upto 10 times
			if ((ABS(gx) > maxOffset) || 
            (ABS(gy) > maxOffset) || 
            (ABS(gz) > maxOffset)) {
				foundBadData = 1;
				ESP_LOGE(TAG, "gyro calib try [%d] bias > %d",inx, maxOffset);
				break;
				}  
			gxAccum  += (int32_t) gx;
			gyAccum  += (int32_t) gy;
			gzAccum  += (int32_t) gz;
			}
		} while (foundBadData && (++numTries < 10));

	// update gyro biases only if calibration succeeded, else use the last saved 
   // values from flash memory. Valid scenario for gyro calibration failing is 
   // when you turn on the unit while flying. So not a big deal.
    if (!foundBadData) {		
		gxBias_ =  (int16_t)( gxAccum / GYRO_NUM_CALIB_SAMPLES);
		gyBias_ =  (int16_t)( gyAccum / GYRO_NUM_CALIB_SAMPLES);
		gzBias_ =  (int16_t)( gzAccum / GYRO_NUM_CALIB_SAMPLES);		
      calib.gxBias = gxBias_;
      calib.gyBias = gyBias_;
      calib.gzBias = gzBias_;
      calib_save();
		}
	ESP_LOGI(TAG,"Num Tries = %d",numTries);
	ESP_LOGI(TAG,"gxBias = %d",gxBias_);
	ESP_LOGI(TAG,"gyBias = %d",gyBias_);
	ESP_LOGI(TAG,"gzBias = %d",gzBias_);
   // reset bandwidth for normal use
   if(mpu9250_writeRegister(CONFIG,GYRO_DLPF_184) < 0){
      ESP_LOGE(TAG,"gyro calib : error resetting bandwidth");
      return -3;
      }
	return (foundBadData ? -4 : 0);
	}


#define MAG_NUM_CALIB_SAMPLES			4000
	
int mpu9250_calibrateMag(void) {
	int16_t mx,my,mz;
	int mxs, mys, mzs, mxMin, myMin,mzMin,mxMax, myMax, mzMax;
	mxMin = myMin = mzMin = 99999;
   mxMax = myMax = mzMax = -99999;
   ESP_LOGI(TAG, "Calibrating magnetometer");
   delayMs(100);
	for (int inx = 0; inx < MAG_NUM_CALIB_SAMPLES; inx++){
		if (mpu9250_getVector(EXT_SENS_DATA_00, true, &mx, &my, &mz) < 0) {
         ESP_LOGE(TAG, "mag calib : error reading data");
         return -1;
         }
      mxs = (mx * (asaX_+128))/256;
      mys = (my * (asaY_+128))/256;
      mzs = (mz * (asaZ_+128))/256;
	   ESP_LOGI(TAG, "[%d] %d %d %d", inx, mxs, mys, mzs);
		if (mxs > mxMax) mxMax = mxs;
      if (mxs < mxMin) mxMin = mxs;
		if (mys > myMax) myMax = mys;
      if (mys < myMin) myMin = mys;
		if (mzs > mzMax) mzMax = mzs;
      if (mzs < mzMin) mzMin = mzs;
      delayMs(10);
		}
	ESP_LOGI(TAG,"mxMin = %d, mxMax = %d",mxMin, mxMax);
	ESP_LOGI(TAG,"myMin = %d, myMax = %d",myMin, myMax);
	ESP_LOGI(TAG,"mzMin = %d, mzMax = %d",mzMin, mzMax);

   mxBias_ = (mxMin+mxMax)/2;
   myBias_ = (myMin+myMax)/2;
   mzBias_ = (mzMin+mzMax)/2;
	mxSens_ = (mxMax - mxMin)/2;
	mySens_ = (myMax - myMin)/2;
	mzSens_ = (mzMax - mzMin)/2;
	
	magXScale_ = 1000.0f/(float)mxSens_;
	magYScale_ = 1000.0f/(float)mySens_;
	magZScale_ = 1000.0f/(float)mzSens_;
   calib.mxBias = mxBias_;
   calib.myBias = myBias_;
   calib.mzBias = mzBias_;
   calib.mxSens = mxSens_;
   calib.mySens = mySens_;
   calib.mzSens = mzSens_;
   calib_save();

#if 0
   lcd_clear();
   lcd_printf(0,0,"x %d %d",mxMin, mxMax);
   lcd_printf(1,0,"y %d %d",myMin, myMax);
   lcd_printf(2,0,"z %d %d",mzMin, mzMax);
   lcd_printf(3,0,"b %d %d %d",mxBias_, myBias_, mzBias_);
   lcd_printf(4,0,"s %d %d %d",mxSens_, mySens_, mzSens_);
#endif

	ESP_LOGI(TAG,"cal : mxBias = %d, myBias = %d, mzBias = %d", mxBias_, myBias_, mzBias_);
	ESP_LOGI(TAG,"cal : mxSens  = %d, mySens = %d, mzSens = %d", mxSens_, mySens_, mzSens_);

   return 1;
	}


int mpu9250_getVector(uint8_t startAddr, int isLittleEndian, int16_t* px, int16_t* py, int16_t* pz) {
   uint8_t buf[6];
   if (mpu9250_readRegisters(startAddr, 6, buf) < 0) {
      return -1;
      }
   if (isLittleEndian) {
   	*px = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
	   *py = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
	   *pz = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);	
      }
   else {
   	*px = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	   *py = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
	   *pz = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);	
      }
   return 1;
	}


int mpu9250_enableDataReadyInterrupt() {
   if (mpu9250_writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
      return -1;
      }  
   if (mpu9250_writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
      return -2;
      }
   return 1;
   }


int mpu9250_disableDataReadyInterrupt() {
   if(mpu9250_writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
      return -1;
      }  
   return 1;
   }


static int mpu9250_writeRegister(uint8_t subAddress, uint8_t data){
	spiSimpleTransaction(_vspi);
	IMU_CS_LO();
	spiTransferByteNL(_vspi, subAddress);
	spiTransferByteNL(_vspi, data);
	IMU_CS_HI();
	spiEndTransaction(_vspi);

   delayMs(10);
   uint8_t buffer[1];
   mpu9250_readRegisters(subAddress,1,buffer);
   return (buffer[0] == data) ? 1 : -1;
   }


static int mpu9250_readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	spiSimpleTransaction(_vspi);
	IMU_CS_LO();
	spiTransferByteNL(_vspi, subAddress | SPI_READ);
   spiTransferBytesNL(_vspi, NULL, dest, count);
	IMU_CS_HI();
	spiEndTransaction(_vspi);	
   return 1;  
   }


static int mpu9250_writeAK8963Register(uint8_t subAddress, uint8_t data){
   // set slave 0 to the AK8963 and set for write
	if (mpu9250_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
      return -1;
      }
   // set the register to the desired AK8963 sub address 
	if (mpu9250_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
      return -2;
      }
   // store the data for write
	if (mpu9250_writeRegister(I2C_SLV0_DO,data) < 0) {
      return -3;
      }
   // enable I2C and send 1 byte
	if (mpu9250_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
      return -4;
      }
	// read the register and confirm
   uint8_t buffer[1];
	if (mpu9250_readAK8963Registers(subAddress,1,buffer) < 0) {
      return -5;
      }
	if(buffer[0] == data) {
  	   return 1;
      } 
   else{
  	   return -6;
      }
   }


static int mpu9250_readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
   // set slave 0 to the AK8963 and set for read
	if (mpu9250_writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
      return -1;
      }
   // set the register to the desired AK8963 sub address
	if (mpu9250_writeRegister(I2C_SLV0_REG,subAddress) < 0) {
      return -2;
      }
   // enable I2C and request the bytes
	if (mpu9250_writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
      return -3;
      }
	cct_delayUs(1000); // takes some time for these registers to fill
   // read the bytes off the MPU9250 EXT_SENS_DATA registers
	uint8_t status = mpu9250_readRegisters(EXT_SENS_DATA_00,count,dest); 
   return status;
   }


//expected 0x71 
static int mpu9250_whoAmI(){
   uint8_t buffer[1];
   if (mpu9250_readRegisters(WHO_AM_I,1,buffer) < 0) {
      return -1;
      }
   return buffer[0];
   }


//expected 0x48 
static int mpu9250_whoAmIAK8963(){
   uint8_t buffer[1];
   if (mpu9250_readAK8963Registers(AK8963_WHO_AM_I,1,buffer) < 0) {
      return -1;
      }
   return buffer[0];
   }

