#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include "common.h"
#include "config.h"
#include "imu.h"
#include "flashlog.h"
#include "ringbuf.h"
#include "kalmanfilter2.h"
#include "kalmanfilter3.h"
#include "kalmanfilter4.h"
#include "kalmanfilter4d.h"


#define KF2    0
#define KF3    1
#define KF4    2
#define KF4D   3

// Build
//    g++ -o kf_run kf_run.cpp kalmanfilter2.cpp kalmanfilter3.cpp 
//    ../../src/sensor/kalmanfilter4.cpp ../../src/sensor/imu.cpp ../../src/sensor/ringbuf.cpp -I../../src/sensor -I. -lm
// Run 
//    ./kf_run -kf2 ./datalog > kf2_results.txt
//    ./kf_run -kf3 ./datalog > kf3_results.txt
//    ./kf_run -kf4 ./datalog > kf4_results.txt
//    ./kf_run -kf4d  ./datalog > kf4d_results.txt

int main(int argc, char* argv[]) {
   int algo_option = -1;
   if (argc != 3) {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
      return -1;
      }
   if (!strcmp(argv[1], "-kf2") ) {
      algo_option = KF2;
      }
   else
   if (!strcmp(argv[1], "-kf3") ) {
      algo_option = KF3;
      }
   else      
   if (!strcmp(argv[1], "-kf4") ) {
      algo_option = KF4;
      }
   else   
   if (!strcmp(argv[1], "-kf4d") ) {
      algo_option = KF4D;
      }
   else {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
      return -1;
      }
	FILE* fp = fopen(argv[2], "rb");
	if (fp == NULL) {
      fprintf(stderr,"Usage : %s -kf2/kf3/kf4/kf4d  ibg_binary_file\n", argv[0]);
		fprintf(stderr,"Error opening %s", argv[1]);
		return(-1);
		}

   IBG_HDR hdr;
   I_RECORD imu;
   B_RECORD baro;
   G_RECORD gps;
   int kalmanFilterInitialized = 0;
   float gx,gy,gz,ax,ay,az,mx, my, mz; 
   float baroAltCm, velNorth, velEast, velDown;
   float iirClimbrateCps, kfClimbrateCps,kfAltitudeCm;
   float glideRatio = 1.0f;
   float zAccelAverage;

	while (!feof(fp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(IBG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(IBG_HDR)) {
         if (hdr.magic == FLASHLOG_IBG_MAGIC) {
      		int imuSize = fread(&imu,1, sizeof(I_RECORD), fp);
            if (imuSize == sizeof(I_RECORD)) {
			      gx = DEG2RAD(imu.gxNEDdps);
			      gy = DEG2RAD(imu.gyNEDdps);
			      gz = DEG2RAD(imu.gzNEDdps);
			      ax = imu.axNEDmG;
			      ay = imu.ayNEDmG;
			      az = imu.azNEDmG;
			      mx = imu.mxNED;
			      my = imu.myNED;
			      mz = imu.mzNED;
               float asqd = ax*ax + ay*ay + az*az;
               // constrain use of accelerometer data to the window [0.75G, 1.25G] for determining
               // the orientation quaternion
		         int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
               int useMag = 1;
               // IMU sensor sampled at 500Hz
		         imu_mahonyAHRSupdate9DOF(useAccel, useMag, IMU_SAMPLE_PERIOD_SECS,gx,gy,gz,ax,ay,az,mx,my,mz);
		         float yawDeg,pitchDeg,rollDeg;
         	   imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&yawDeg, (float*)&pitchDeg, (float*)&rollDeg);
               float gravityCompensatedAccel = imu_gravityCompensatedAccel(ax,ay,az, q0, q1, q2, q3);
               ringbuf_addSample(gravityCompensatedAccel); 
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(B_RECORD), fp);
               if ((baroSize == sizeof(B_RECORD)) && hdr.baroFlags) {
                  baroAltCm = (float) baro.heightMSLcm;
                  switch (algo_option) {
                     case 0 : // KF2
                     default:
                     if (kalmanFilterInitialized == 0) {
                        kalmanFilter2_configure(KF_Z_MEAS_VARIANCE, ((float)KF_ACCEL_VARIANCE)*1000.0f, baroAltCm, 0.0f);
                        kalmanFilterInitialized = 1;
                        }
                     kalmanFilter2_predict(KF_ACCEL_VARIANCE*1000.0f, KF_SAMPLE_PERIOD_SECS);
                     kalmanFilter2_update(baroAltCm, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
                     break;

                     case 1 : // KF3
                     if (kalmanFilterInitialized == 0) {
                        kalmanFilter3_configure(KF_Z_MEAS_VARIANCE, ((float)KF_ACCEL_VARIANCE)*1000.0f, baroAltCm, 0.0f);
                        kalmanFilterInitialized = 1;
                        }
                     // acceleration data used in the predict phase
		      		 zAccelAverage = ringbuf_averageOldestSamples(10); 
                     kalmanFilter3_predict(zAccelAverage, KF_SAMPLE_PERIOD_SECS);
                     kalmanFilter3_update(baroAltCm, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
                     break;

                     case 2 ://  KF4
                     if (kalmanFilterInitialized == 0) {
                        kalmanFilter4_configure(((float)KF_ACCEL_VARIANCE)*1000.0f, baroAltCm, 0.0f, 0.0f);
                        kalmanFilterInitialized = 1;
                        }
                     // acceleration data used in the update phase
		      		 zAccelAverage = ringbuf_averageNewestSamples(10); 
                     kalmanFilter4_predict(KF_SAMPLE_PERIOD_SECS);
                     kalmanFilter4_update(baroAltCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
                     break;

                     case 3 : // KF4D
                     if (kalmanFilterInitialized == 0) {
                        kalmanFilter4d_configure(((float)KF_ACCEL_VARIANCE)*1000.0f, ((float)KF_ADAPT_DEFAULT)/100.0f, baroAltCm, 0.0f, 0.0f);
                        kalmanFilterInitialized = 1;
                        }
                     // acceleration data used in the update phase
		      		 zAccelAverage = ringbuf_averageNewestSamples(10); 
                     kalmanFilter4d_predict(KF_SAMPLE_PERIOD_SECS);
                     kalmanFilter4d_update(baroAltCm, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
                     break;
                     }
                  // use damped climbrate for lcd display and for glide ratio computation
                  iirClimbrateCps = iirClimbrateCps*0.9f + 0.1f*kfClimbrateCps; 
                  } 
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(G_RECORD), fp);
               if (gpsSize == sizeof(G_RECORD)) {
                  velNorth = (float) gps.velNorthmmps;
                  velEast = (float) gps.velEastmmps;
                  velDown = (float) gps.velDownmmps;
                  float velHorz = (float)sqrt((double)velNorth*velNorth + (double)velEast*velEast);
                  if (velDown > 0.0f) {
                     float glide = velHorz/velDown;
                     glideRatio = glideRatio*0.97f + glide*0.03f;
                     }
                  velHorz /= 10.0f; // in cm/s
                  float velkph = (velHorz*3600.0f)/100000.0f;
                  } 
               }
            }
         else {
            printf("\r\nError : magic not found\r\n");
            break;
            }
         }
      else {
        // printf("\r\nDid not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(fp);
   }

