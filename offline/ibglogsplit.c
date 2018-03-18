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
 


int main(int argc, char* argv[]) {
   IBG_HDR hdr;
   I_RECORD imu;
   B_RECORD baro;
   G_RECORD gps;
	if (argc != 3) {
		printf("Usage : ibglogsplit <binaryLogFile> <maxDropoutMs>\r\n e.g. .\\ibglogsplit datalog 200\r\n");
		return -1;
		}
	FILE* ifp = fopen(argv[1], "rb");
	FILE* ofp = NULL;
	int fileCounter = 0;
	char szFileName[20];
	if (ifp == NULL) {
		printf("error opening %s", argv[1]);
		return(-1);
		}
	int maxDropoutMilliseconds = atoi(argv[2]);
	int towms = 0;

	while (!feof(ifp)) {	
      hdr.magic = 0;
		int hdrSize = fread(&hdr,1, sizeof(IBG_HDR), ifp);
      int numRecordBytes = 0;
		if (hdrSize == sizeof(IBG_HDR)) {
         if (hdr.magic == FLASHLOG_IBG_MAGIC) {
      		int imuSize = fread(&imu,1, sizeof(I_RECORD), ifp);
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(B_RECORD), ifp);
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(G_RECORD), ifp);
			      int timediff = abs(gps.timeOfWeekmS - towms);
			      if (timediff > maxDropoutMilliseconds) {
				      printf("new stream found\r\n");
				      if (ofp) {
					      fclose(ofp);					
					      }
				      sprintf(szFileName, "%s_%02d",argv[1], fileCounter++);
				      ofp = fopen(szFileName, "wb");
				      if (ofp == NULL) {
					      printf("error opening %s", argv[1]);
					      fclose(ifp);
					      return(-1);
					      }  
				      }
      			towms = gps.timeOfWeekmS;        
               } 
               
			   fwrite((uint8_t*)&hdr,1, sizeof(IBG_HDR), ofp);
			   fwrite((uint8_t*)&imu,1, sizeof(I_RECORD), ofp);
            if (hdr.baroFlags || hdr.gpsFlags) {
			      fwrite((uint8_t*)&baro,1, sizeof(B_RECORD), ofp);
               }
            if (hdr.gpsFlags) {
			      fwrite((uint8_t*)&gps,1, sizeof(G_RECORD), ofp);
               }
            }
         else {
            if (hdr.magic == FLASHLOG_GPS_MAGIC) {
               printf("\r\nData log is a GPS track file, not an IBG log file\r\n");
               fclose(ifp);
               if (ofp) fclose(ofp);
               return -2;
               }
            else {
               printf("\r\nError : magic not found, unknown type of file\r\n");
               fclose(ifp);
               if (ofp) fclose(ofp);
               return -3;
               }
            }
         }
      else {
         printf("\r\nDid not read hdr len bytes\r\n");
         break;
         }
      }
   fclose(ifp);
   if (ofp) fclose(ofp);
   return 0;
   }

