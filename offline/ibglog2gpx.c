#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct LOG_RECORD_ {
	uint32_t	flags;
	uint32_t  	gpsTimeOfWeekMs;
	int32_t 	gpsLongitudeDeg7;
	int32_t 	gpsLatitudeDeg7;	
	int32_t		gpsAltMslMm;
	uint32_t	gpsVertAccuracyMm;
	int32_t		gpsVelocityNorthMmps;
	int32_t		gpsVelocityEastMmps;
	int32_t		gpsVelocityDownMmps;
	uint32_t	gpsVelAccuracyMmps;
	int32_t   	baroAltCm;
	int32_t 	gxned;
	int32_t 	gyned;
	int32_t 	gzned;
	int32_t 	axned;
	int32_t 	ayned;
	int32_t 	azned;
	int32_t 	mxned;
	int32_t 	myned;
	int32_t 	mzned;
} LOG_RECORD;
 
LOG_RECORD rec;


const char szGpxHeader[] = "<?xml version=\"1.0\"?>\n<gpx version=\"1.1\">\r\n";
const char szGpxOpen[]	 = "<trk><trkseg>\r\n";
const char szGpxClose[]	 = "</trkseg></trk></gpx>\r\n";

int main(int argc, char* argv[]) {
	if (argc != 7) {
		printf("Usage : log2gpx <binaryLogFile> <year> <month> <day> <hour24> <minute>\r\n");
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
	
	int year = atoi(argv[2]);
	int month = atoi(argv[3]);
	int day = atoi(argv[4]);
	int hour = atoi(argv[5]);
	int minute = atoi(argv[6]);
	int towms = 0;
	float second = 0.0f;
	printf("\r\nSaving to %s, log starts at %d/%02d/%02d %02d:%02d:00\r\n", szofp, year, month, day, hour, minute);

	fprintf(ofp,"%s",szGpxHeader);
	fprintf(ofp,"%s",szGpxOpen);
	float lon, lat, alt;
	
	while (!feof(ifp)) {	
		int readbytes = fread((uint8_t*)&rec,1, sizeof(LOG_RECORD), ifp);
		if (readbytes == sizeof(LOG_RECORD)) {
			int intervalMs = abs(rec.gpsTimeOfWeekMs - towms);
			if (intervalMs) {
				lon = (float)rec.gpsLongitudeDeg7/10000000.0f;
				lat = (float)rec.gpsLatitudeDeg7/10000000.0f;
				alt = (float)rec.gpsAltMslMm/1000.0f;
				if (towms) {
					second += (float)intervalMs/1000.0f;
					}
				towms = rec.gpsTimeOfWeekMs;
				while (second > 60.0f) {
					second -= 60.0f;
					minute++;
					}
				while (minute > 59) {
					minute -=60;
					hour++;
					}
				fprintf(ofp,"<trkpt lat=\"%f\" lon=\"%f\"> <ele>%f</ele> <time>%d-%02d-%02dT%02d:%02d:%fZ</time> </trkpt>\r\n",
      lat,lon,alt,year,month,day,hour,minute,second);
				}
			}
		}
	
	fclose(ifp);
	fprintf(ofp,"%s",szGpxClose);
	fclose(ofp);
	return 0;
	}

	


