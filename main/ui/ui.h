#ifndef UI_H_
#define UI_H_

#include "gps.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIN_VELOCITY_KPH	0
#define MAX_VELOCITY_KPH	999

#define MIN_GLIDE_RATIO		0
#define MAX_GLIDE_RATIO		999

#define MIN_ALTITUDE_M 		-999
#define MAX_ALTITUDE_M 		9999

#define MIN_DISTANCE_M   	0
#define MAX_DISTANCE_M 	   999999

#define MIN_CLIMBRATE_CPS   (-2500)
#define MAX_CLIMBRATE_CPS    (2500)

#define MIN_COMPASS_HEADING		0
#define MAX_COMPASS_HEADING		359

#define FLOAT_DEG(val)  (((float)val)/10000000.0f)
#define RANGE_360(d)  (((d)+360)%360) // 0 - 359 degrees

#define SEL_CLIMB_THRESHOLD         0
#define SEL_ZERO_THRESHOLD          1
#define SEL_SINK_THRESHOLD          2
#define SEL_CROSSOVER_THRESHOLD     3
#define SEL_VARIO_DISPLAY_IIR       4
#define SEL_ACCEL_VAR               5
#define SEL_ZMEAS_VAR               6
#define SEL_UTC_OFFSET              7
#define SEL_BKLIGHT_SECS            8
#define SEL_TRACK_THRESHOLD         9
#define SEL_TRACK_INTERVAL          10
#define SEL_GLIDE_IIR               11
#define SEL_GPS_STABLE_DOP          12
#define SEL_GYRO_OFFSET_MAX         13
#define SEL_MAG_DECLINATION         14
#define SEL_SPKR_VOL                15
#define SEL_LOG_TYPE                16
#define SEL_WPT_RADIUS              17
#define SEL_ALTITUDE_DISPLAY        18
#define SEL_BTMSG_TYPE				19
#define SEL_BTMSG_FREQ				20
#define SEL_LCD_CONTRAST			21


typedef struct TRACK_ {
   int   year;
   int   month;
   int   day;
   int   hour;
   int   minute;
   float startLatdeg;
   float startLondeg;
   int32_t startAltm;
   int32_t maxAltm;
   float maxClimbrateCps;
   float maxSinkrateCps;
   int32_t startTowmS;
   int32_t distanceFromStartm;
   int32_t elapsedHours;
   int32_t elapsedMinutes;
   int     nextWptInx;
} TRACK;



void ui_updateFlightDisplay(NAV_PVT* pn, TRACK* ptrk);
int  ui_saveFlightLogSummary(NAV_PVT* pn, TRACK* pTrk);
bool ui_optionsEventHandler(void);
void ui_screenInit();
void ui_displayOptions(void);


extern bool IsSpeakerEnabled;
extern bool IsBluetoothEnabled;
extern bool IsRouteActive;
extern bool IsGpsFixStable;
extern bool IsTrackActive;
extern bool IsLcdBkltEnabled;
extern bool IsLogging;
extern bool IsServer;
extern bool IsFlashDisplayRequired;
extern bool IsGpsHeading;
extern bool EndTrack;

extern int  SupplyVoltageMV;
extern int32_t GpsCourseHeadingDeg;
extern int32_t CompassHeadingDeg;

extern const char szLogType[3][5];
extern const char szAltDisplayType[2][5];
extern const char szBtMsgType[2][4];

#ifdef __cplusplus
}
#endif

#endif

