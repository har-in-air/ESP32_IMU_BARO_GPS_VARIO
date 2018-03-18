#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "flashlog.h"

int main(int argc, char* argv[]) {
   IBG_HDR  hdr;
   I_RECORD imu;
   B_RECORD baro;
   G_RECORD gps;
   T_RECORD trkpt;

	if (argc != 4) {
		printf("Usage : logsplit <binaryLogFile> <maxIBGDropoutSecs> <maxGpsDropoutSecs>\r\n e.g. .\\logsplit datalog 1 10\r\n");
		return -1;
		}
	FILE* ifp = fopen(argv[1], "rb");
	FILE* ofp = NULL;
	int ibgFileCounter = 0;
	int gpsFileCounter = 0;
	char szFileName[20];
	if (ifp == NULL) {
		printf("error opening %s", argv[1]);
		return(-1);
		}
	int maxIBGDropoutSecs = atoi(argv[2]);
   int maxGpsDropoutSecs = atoi(argv[3]);
	int ibgTime = 0;
   int gpsTime = 0;

	while (!feof(ifp)) {	
      hdr.magic = 0;
      // works for both IBG_HDR and GPS_HDR
		int hdrSize = fread(&hdr,1, sizeof(IBG_HDR), ifp);
		if (hdrSize == sizeof(IBG_HDR)) { 
         if (hdr.magic == FLASHLOG_IBG_MAGIC) {
      		int imuSize = fread(&imu,1, sizeof(I_RECORD), ifp);
            if (hdr.baroFlags || hdr.gpsFlags) {
      		   int baroSize = fread(&baro,1, sizeof(B_RECORD), ifp);
               }
            if (hdr.gpsFlags) {
      		   int gpsSize = fread(&gps,1, sizeof(G_RECORD), ifp);
			      int timediffmS = abs(gps.timeOfWeekmS - ibgTime);
               // note : checking the timediff from the last ibgTime stamp works 
               // both for splitting ibg logs and ibg logs with gps log in between.
			      if (timediffmS > (maxIBGDropoutSecs*1000)) {
				      if (ofp) {
					      fclose(ofp);					
					      }
				      sprintf(szFileName, "%s_ibg_%02d",argv[1], ibgFileCounter++);
				      printf("new ibg log %s\r\n", szFileName);
				      ofp = fopen(szFileName, "wb");
				      if (ofp == NULL) {
					      printf("error opening %s", argv[1]);
					      fclose(ifp);
					      return(-1);
					      }  
				      }
      			ibgTime = gps.timeOfWeekmS;        
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
         else 
         if (hdr.magic == FLASHLOG_GPS_MAGIC) {
      		int trkSize = fread(&trkpt,1, sizeof(T_RECORD), ifp);
            int newGpsTime = trkpt.utcDay*86400 + trkpt.utcHour*3600 + trkpt.utcMinute*60 + trkpt.utcSecond;
            if (abs(newGpsTime - gpsTime) > maxGpsDropoutSecs) {
   			   if (ofp) {
	   		      fclose(ofp);					
	   			   }
	   		   sprintf(szFileName, "%s_gps_%02d",argv[1], gpsFileCounter++);
			      printf("new gps log %s\r\n", szFileName);
	   		   ofp = fopen(szFileName, "wb");
	   		   if (ofp == NULL) {
	   			   printf("error opening %s", argv[1]);
	   			   fclose(ifp);
	   			   return(-2);
	   			   } 
               } 
            gpsTime = newGpsTime;
   			fwrite((uint8_t*)&hdr,1, sizeof(GPS_HDR), ofp);
            fwrite((uint8_t*)&trkpt,1, sizeof(T_RECORD), ofp);
            }
         else {
            printf("\r\nError : unknown magic value in header\r\n");
            if (ifp) fclose(ifp);
            if (ofp) fclose(ofp);
            return -3;
            }
         }
      else {
         printf("\r\nDid not read full header\r\n");
         if (ifp) fclose(ifp);
         if (ofp) fclose(ofp);
         return 0;
         }
      }

   if (ifp) fclose(ifp);
   if (ofp) fclose(ofp);
   return 0;
   }

