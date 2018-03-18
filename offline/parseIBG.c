#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



#define FLASHLOG_IBG_MAGIC  0xA55A
#define FLASHLOG_GPS_MAGIC  0x9043

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
 
typedef struct FLASHLOG_GPS_RECORD_ {
   uint16_t magic;
	uint16_t	utcYear;
	uint8_t	utcMonth;
	uint8_t	utcDay;
	uint8_t	utcHour;
	uint8_t	utcMinute;
	uint8_t	utcSecond;
	int32_t	nanoSeconds;
	int32_t 	lonDeg7;
	int32_t 	latDeg7;	
	int32_t	heightMSLmm;
} FLASHLOG_GPS_RECORD;



// if there is only an IMU record, size of record = HDR + IMU
// if there is a BARO record, size of record = HDR + IMU + BARO
// if there is a GPS record, size of record = HDR + IMU + BARO + GPS even if BARO record is invalid

int getNumRecords(char* szFileName) {
   int imuRecordCounter = 0;
   int baroRecordCounter = 0;
   int gpsRecordCounter = 0;
	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return(-1);
		}

	while (!feof(fp)) {	
      IBG_HDR hdr;
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(IBG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(IBG_HDR)) {
         if (hdr.magic == FLASHLOG_IBG_MAGIC) {
            numRecordBytes = sizeof(I_RECORD);
            if (hdr.baroFlags || hdr.gpsFlags) numRecordBytes += sizeof(B_RECORD);
            if (hdr.gpsFlags) numRecordBytes += sizeof(G_RECORD);
            if (fseek(fp, numRecordBytes, SEEK_CUR)) {
               printf("Fseek past end of file\r\n");
               break;
               }
            imuRecordCounter++;
            if (hdr.baroFlags) baroRecordCounter++;
            if (hdr.gpsFlags) gpsRecordCounter++;
            }
         else {
            printf("Error : magic not found\r\n");
            break;
            }
         }
      else {
         printf("Did not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(fp);

   printf("Found %d imu records\r\n", imuRecordCounter);
   printf("Found %d baro records\r\n", baroRecordCounter);
   printf("Found %d gps records\r\n\r\n", gpsRecordCounter);
   }



int printRecords(char* szFileName) {
	FILE* fp = fopen(szFileName, "rb");
	if (fp == NULL) {
		printf("error opening %s", szFileName);
		return(-1);
		}

   IBG_HDR hdr;
   I_RECORD imu;
   B_RECORD baro;
   G_RECORD gps;

	while (!feof(fp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(IBG_HDR), fp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(IBG_HDR)) {
         if (hdr.magic == FLASHLOG_IBG_MAGIC) {
      		int imuSize = fread(&imu,1, sizeof(I_RECORD), fp);
            if (imuSize == sizeof(I_RECORD)) {
               printf("IMU : %f %f %f %f %f %f\r\n", 
                  imu.gxNEDdps, imu.gyNEDdps, imu.gzNEDdps, 
                  imu.axNEDmG, imu.ayNEDmG, imu.azNEDmG);
               } 
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(B_RECORD), fp);
               if (baroSize == sizeof(B_RECORD)) {
                  printf("BARO : %d\r\n",baro.heightMSLcm); 
                  } 
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(G_RECORD), fp);
               if (gpsSize == sizeof(G_RECORD)) {
                  printf("GPS : %d %f %f %d\r\n", gps.timeOfWeekmS, (float)gps.lonDeg7/10000000.0f,(float)gps.latDeg7/10000000.0f,
                     gps.heightMSLmm/10); 
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


// Structs seem to be packed without using any special compiler directives
// Using gcc, ubuntu 16.04, AMD x64 
// gcc -o parseData parseData.c
//  
// And ESP32 is also little endian, so no problem reading the structures from the binary file.

int main(int argc, char* argv[]) {
   if (argc != 2) {
      printf("usage : %s <ibg binary file>\r\n", argv[0]);
      return -1;
      }

   printf("sizeof IBG_HDR = %lu\r\n",  sizeof(IBG_HDR));
   printf("sizeof I_RECORD = %lu\r\n",  sizeof(I_RECORD));
   printf("sizeof FLASHLOG_IBG_RECORD = %lu\r\n\r\n",  sizeof(FLASHLOG_IBG_RECORD));

   getNumRecords(argv[1]);

   printRecords(argv[1]);
   return 0;
	}

