#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "imu.h"



#define FLASHLOG_IBG_MAGIC  0xA55A

typedef struct IBG_HDR_ {
	uint16_t	magic; 
   uint8_t  gpsFlags; 
   uint8_t  baroFlags;
} IBG_HDR;

typedef struct I_RECORD_ {
	float 	gxNEDdps;
	float 	gyNEDdps;
	float 	gzNEDdps;
	float 	axNEDmG;
	float 	ayNEDmG;
	float 	azNEDmG;
	float 	mxNED;
	float 	myNED;
	float 	mzNED;
   } I_RECORD;

typedef struct B_RECORD_ {
   int32_t  heightMSLcm;
} B_RECORD;

typedef struct G_RECORD_ {
	uint32_t timeOfWeekmS;
	int32_t 	lonDeg7;
	int32_t 	latDeg7;	
	int32_t	heightMSLmm;
	uint32_t	vertAccuracymm;
	int32_t	velNorthmmps;
	int32_t	velEastmmps;
	int32_t	velDownmmps;
	uint32_t	velAccuracymmps;
} G_RECORD;

typedef struct FLASHLOG_IBG_RECORD_ {
   IBG_HDR   hdr;
   I_RECORD  imu;
   B_RECORD  baro;
   G_RECORD  gps;
} FLASHLOG_IBG_RECORD;
 
 

int main(int argc, char* argv[]) {
   if (argc != 2) {
      printf("usage : %s <ibg binary file>\r\n", argv[0]);
      return -1;
      }
	FILE* fp = fopen(argv[1], "rb");
	if (fp == NULL) {
		printf("error opening %s", argv[1]);
		return(-1);
		}

   IBG_HDR hdr;
   I_RECORD imu;
   B_RECORD baro;
   G_RECORD gps;
   float gx,gy,gz,ax,ay,az,mx, my, mz; 

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
		         imu_mahonyAHRSupdate9DOF(useAccel, useMag, 0.002,gx,gy,gz,ax,ay,az,mx,my,mz);
		         float yawDeg,pitchDeg,rollDeg;
         	   imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&yawDeg, (float*)&pitchDeg, (float*)&rollDeg);
			      printf("%f, %f, %f, %f, %f, %f, %f\r\n", q0, q1, q2, q3, yawDeg, pitchDeg, rollDeg);
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(B_RECORD), fp);
               if (baroSize == sizeof(B_RECORD)) {
                  // skip past baro record
                  } 
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(G_RECORD), fp);
               if (gpsSize == sizeof(G_RECORD)) {
                  // skip past gps record
                  } 
               }
            }
         else {
            printf("\r\nError : magic not found\r\n");
            break;
            }
         }
      else {
         printf("\r\nDid not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(fp);
   }

