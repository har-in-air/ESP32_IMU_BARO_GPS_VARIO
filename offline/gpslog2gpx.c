#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "flashlog.h"

FLASHLOG_GPS_RECORD rec;

const char szGpxHeader[] = "<?xml version=\"1.0\"?>\n<gpx version=\"1.1\">\r\n";
const char szGpxOpen[]	 = "<trk><trkseg>\r\n";
const char szGpxClose[]	 = "</trkseg></trk></gpx>\r\n";

int main(int argc, char* argv[]) {
	if (argc != 2) {
		printf("Usage : %s <binaryLogFile>\r\n",argv[0]);
		return -1;
		}
	FILE* ifp = fopen(argv[1], "rb");
	if (ifp == NULL) {
		printf("error opening %s to read", argv[1]);
		return(-1);
		}
	char szofp[20];
	sprintf(szofp, "%s.gpx", argv[1]);
	FILE* ofp = fopen(szofp, "w");
	if (ifp == NULL) {
		printf("error opening %s to write", szofp);
		return(-1);
		}
	
	int year, month, day, hour, minute, second, nanoSeconds;
	printf("\r\nSaving to %s\r\n", szofp);

	fprintf(ofp,"%s",szGpxHeader);
	fprintf(ofp,"%s",szGpxOpen);
	float lon, lat, alt;
	
	while (!feof(ifp)) {	
		int readbytes = fread((uint8_t*)&rec,1, sizeof(FLASHLOG_GPS_RECORD), ifp);
		if (readbytes == sizeof(FLASHLOG_GPS_RECORD)) {
				lon = (float)rec.trkpt.lonDeg7/10000000.0f;
				lat = (float)rec.trkpt.latDeg7/10000000.0f;
				alt = (float)rec.trkpt.heightMSLmm/1000.0f;
				fprintf(ofp,"<trkpt lat=\"%f\" lon=\"%f\"> <ele>%f</ele> <time>%d-%02d-%02dT%02d:%02d:%dZ</time> </trkpt>\r\n",    lat,lon,alt,rec.trkpt.utcYear,rec.trkpt.utcMonth,rec.trkpt.utcDay,rec.trkpt.utcHour,rec.trkpt.utcMinute,rec.trkpt.utcSecond);
				}
			}
		
	
	fclose(ifp);
	fprintf(ofp,"%s",szGpxClose);
	fclose(ofp);
	return 0;
	}

	


