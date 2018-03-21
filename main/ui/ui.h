#ifndef UI_H_
#define UI_H_

#include "gps.h"

#define MIN_VELOCITY_KPH	0
#define MAX_VELOCITY_KPH	990

#define MIN_GLIDE_RATIO		0
#define MAX_GLIDE_RATIO		99

#define MIN_ALTITUDE_M 		-999
#define MAX_ALTITUDE_M 		9999

#define MIN_DISTANCE_M   	0
#define MAX_DISTANCE_M 	   999999

#define MIN_CLIMBRATE_CPS   (-2500)
#define MAX_CLIMBRATE_CPS    (2500)

#define MIN_COMPASS_HEADING		0
#define MAX_COMPASS_HEADING		359

#define FLOAT_DEG(val)  (((float)val)/10000000.0f)

typedef struct TRACK_ {
   float startLatDeg;
   float startLonDeg;
   float startAltm;
   int32_t maxAltm;
   float maxClimbrateCps;
   float maxSinkrateCps;
   int32_t startTowmS;
   int32_t distanceFromStartm;
   int     nextWptInx;
} TRACK;


void ui_printPosDOP(int page, int col, int dop);				
void ui_printBatteryStatus(int page, int col, int bV);
void ui_printSpkrStatus(int page, int col, int bAudioEn);
void ui_printAltitude(int page, int col, int32_t nAlt);
void ui_printDistance(int page, int col, int distanceM) ;
void ui_printGlideRatio(int page, int col, int nGr);
void ui_printVelocity(int page, int col, int nVel);
void ui_printClimbRate(int page, int col, int nCps);
void ui_printBearingAnalog(int page, int col, int bearing);
void ui_printCompassHeadingAnalog(int isGps, int page, int col, int velkph, int heading);
void ui_printRealTime(int page, int col, int nHrs, int nMin);
void ui_printTrackTime(int page, int col, int nHrs, int nMin);
void ui_printLongitude(int page, int col, int32_t nLon);
void ui_printLatitude(int page, int col, int32_t nLat);
void ui_updateFlightDisplay(NAV_PVT* pn, TRACK* ptrk);
void ui_printRouteSegment(int page, int col, int start, int end);

extern bool IsSpeakerEnabled;
extern bool IsRouteActive;
extern bool IsGpsFixStable;
extern bool IsTrackActive;
extern bool IsLcdBkltEnabled;
extern bool IsLogging;
extern bool IsServer;
extern bool IsFlashDisplayRequired;
extern bool IsGpsHeading;

extern volatile float YawDeg;
extern volatile float IIRClimbrateCps;

#endif

