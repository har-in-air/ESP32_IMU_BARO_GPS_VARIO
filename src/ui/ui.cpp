#include "common.h"
#include <FS.h>
#include <LITTLEFS.h>
#include <math.h>
#include "config.h"
#include "drv/btn.h"
#include "drv/cct.h"
#include "drv/audio.h"
#include "sensor/madc.h"
#include "sensor/gps.h"
#include "sensor/ms5611.h"
#include "nv/options.h"
#include "ui.h"
#include "lcd7565.h"
#include "route.h"

static const char* TAG = "ui";

bool IsRouteActive = false;
bool IsSpeakerEnabled  = true;
bool IsGpsFixStable = false;
bool IsGpsTrackActive = false;
bool IsLcdBkltEnabled = false;
bool IsLoggingIBG = false;
bool IsFlashDisplayRequired = false;
bool IsGpsHeading = true;
bool EndGpsTrack = false;
bool IsBluetoothEnabled = false;

int SupplyVoltageMV = 0;
int32_t GpsCourseHeadingDeg;
int32_t CompassHeadingDeg;

const char szLogType[3][5]    = {"NONE", " GPS", " IBG"};
const char szAltDisplayType[2][5] = {" GPS", "BARO"};
const char szBtMsgType[2][4]  = {"LK8", "XCT"};

static int ParChanged = 0;
static int ScreenParOffset = 0;
static int ParDisplaySel = 0;
static int ParSel = 0;

static void ui_printSupplyVoltage(int page, int col, int bV);
static void ui_printBatteryStatus(int page, int col, int bV);
static void ui_printSpkrStatus(int page, int col, bool bAudioEn);
static void ui_printBluetoothStatus(int page, int col, bool bBluetoothEn);
static void ui_printAltitude(int page, int col, int32_t nAlt);
static void ui_printDistance(int page, int col, int distanceM) ;
static void ui_printGlideRatio(int page, int col, int nGr);
static void ui_printVelocity(int page, int col, int nVel);
static void ui_printClimbRate(int page, int col, int nCps);
static void ui_printBearingAnalog(int page, int col, int velkph, int bearing);
static void ui_printHeadingAnalog(int page, int col, int velkph, int heading);
static void ui_printRealTime(int page, int col, int nHrs, int nMin);
static void ui_printTrackTime(int page, int col, int nHrs, int nMin);
static void ui_printLongitude(int page, int col, int32_t nLon);
static void ui_printLatitude(int page, int col, int32_t nLat);
static void ui_printRouteSegment(int page, int col, int start, int end);
static void ui_calcTrackElapsedTime(int32_t startmS, int32_t currentmS, int32_t* pHrs, int32_t* pMins);
static void ui_alarmWaypointReached();

static void ui_printLatitude(int page, int col, int32_t nLat) {
	char szBuf[12];
	int32_t  nL;
	int inx;
	nL = ABS(nLat)/100;
	sprintf(szBuf,"%c%d", (nLat >= 0 ? 'N' : 'S'), nL/100000);
	lcd_set_frame_pos(page,col);
	lcd_print_sz_lnum(page,col,szBuf);
	sprintf(szBuf,"%05d", nL%100000);
	inx = 0;
	while (szBuf[inx]) {
		lcd_put_char(szBuf[inx++]);
        }
   }


static void ui_printLongitude(int page, int col, int32_t nLon) {
	char  szBuf[12];
    int32_t  nL;
    int inx;
    nL = ABS(nLon)/100;
    sprintf(szBuf,"%c%d", (nLon >= 0 ? 'E' : 'W'), nL/100000);
	lcd_set_frame_pos(page,col);
	lcd_print_sz_lnum(page,col,szBuf);
    sprintf(szBuf,"%05d", nL%100000);
    inx = 0;
    while (szBuf[inx]) {
		lcd_put_char(szBuf[inx++]);
        }
   }


static void ui_printAltitude(int page, int col, int32_t altm) {
	char szBuf[5];
	CLAMP(altm,MIN_ALTITUDE_M,MAX_ALTITUDE_M);
	if (altm == 9999) {
		sprintf(szBuf,"----");
		}
	else {
		sprintf(szBuf,"%4d",altm);
		}
	lcd_print_sz_lnum(page,col,szBuf);
   }


static void ui_printDistance(int page, int col, int distanceM) {
	char szBuf[4];
	CLAMP(distanceM,MIN_DISTANCE_M,MAX_DISTANCE_M);
	distanceM = (distanceM + 5)/10;
	lcd_set_frame_pos(page,col+40);
	lcd_put_char(48+ (distanceM%10));
	distanceM = distanceM/10;
	lcd_set_frame_pos(page,col+33);
	lcd_put_char(48+(distanceM%10));
	distanceM = distanceM/10;
	sprintf(szBuf,"%3d", distanceM);
	lcd_print_sz_lnum(page,col,szBuf);
	}


static void ui_printClimbRate(int page, int col, int nCps) {
	char  sign;
	int  dec,frac;
	char  szBuf[4];

	szBuf[1] = ' ';
	szBuf[0] = ' ';
	CLAMP(nCps, MIN_CLIMBRATE_CPS, MAX_CLIMBRATE_CPS);
	nCps = (nCps < 0 ?  (nCps-5)/10 : (nCps+5)/10);
	if (nCps > 0){
		sign = '+';
		}
	else
	if (nCps < 0){
		sign = '-';
		nCps = -nCps;
		}
	else {
		sign = ' ';	
		}	
	szBuf[3] = '\0';
	lcd_set_frame_pos(page,col+33);
	if (sign == ' ') {
		lcd_put_char('0');
	  	szBuf[2] = '0';
     	}
	else {
		dec = nCps/10;
		frac = nCps%10;
		lcd_put_char(frac+48);
		if (dec == 0) {
	  		szBuf[2] = '0';
	  		szBuf[1] = sign;
			}
		else {
      	szBuf[2] = (dec%10) + 48; 
	  		dec = dec/10;
			if (dec == 0) {
				szBuf[1] = sign;
				}
			else {
				szBuf[1] = dec + 48;
				szBuf[0] = sign;
				}
			}
		}
	lcd_print_sz_lnum(page,col,szBuf);
  	}


static void ui_printVelocity(int page, int col, int nVel) {
	char  szBuf[4];
	CLAMP(nVel, 0, MAX_VELOCITY_KPH);
	sprintf(szBuf, "%3d", nVel);
	lcd_print_sz_lnum(page,col,szBuf);
	}


static void ui_printGlideRatio(int page, int col, int nGr) {
	char  szBuf[3];
	szBuf[2] = '\0';
	if (nGr > 999) {
	   lcd_set_frame_pos(page,col+24);
	   lcd_put_char(' ');
		szBuf[1] = '+';
		szBuf[0] = ' ';
		}
	else{
	   lcd_set_frame_pos(page,col+24);
	   lcd_put_char(48+(nGr%10));
	   nGr = nGr/10;
		szBuf[1] = (nGr%10) + 48;
		nGr = nGr/10;
		szBuf[0] = nGr ? (nGr+48) : ' ';
	   }
	lcd_print_sz_lnum(page,col,szBuf);
	}


#include "compass.txt"

static uint8_t compassBuf[128];

static void ui_printHeadingAnalog(int page, int col, int velkph, int headingdeg) {
	int nrow,ncol,inx;
	int tblOffset = 0;
	lcd_set_frame_pos(page - 1, col+13);

	// caret on top of the heading display
	// bar => gps course over ground (direction of motion)
	// diamond => magnetic compass heading (direction unit is facing)
   if (IsGpsHeading) {
      FrameBuf[128*FramePage + FrameCol+1] = 0x3C;
      FrameBuf[128*FramePage + FrameCol+2] = 0x7C;
      FrameBuf[128*FramePage + FrameCol+3] = 0x3C;  
      // if very low velocity, gps course over ground heading is uncertain, do not display
	   if (velkph < 2) { 
		   tblOffset = 16;
	   	}
      }
   else {
      FrameBuf[128*FramePage + FrameCol]   = 0x10;
      FrameBuf[128*FramePage + FrameCol+1] = 0x38;
      FrameBuf[128*FramePage + FrameCol+2] = 0x7C;
      FrameBuf[128*FramePage + FrameCol+3] = 0x38;
      FrameBuf[128*FramePage + FrameCol+4] = 0x10;
      }

   if (!tblOffset)  {
		if (headingdeg <= 11) tblOffset = 0;
		else
		if (headingdeg <= 34) tblOffset = 1;
		else
		if (headingdeg <= 56) tblOffset = 2;
		else
		if (headingdeg <= 79) tblOffset = 3;
		else
		if (headingdeg <= 101) tblOffset = 4;
		else
		if (headingdeg <= 124) tblOffset = 5;
		else
		if (headingdeg <= 146) tblOffset = 6;
		else
		if (headingdeg <= 169) tblOffset = 7;
		else
		if (headingdeg <= 191) tblOffset = 8;
		else
		if (headingdeg <= 214) tblOffset = 9;
		else
		if (headingdeg <= 236) tblOffset = 10;
		else
		if (headingdeg <= 259) tblOffset = 11;
		else
		if (headingdeg <= 281) tblOffset = 12;
		else
		if (headingdeg <= 304) tblOffset = 13;
		else
		if (headingdeg <= 326) tblOffset = 14;
		else
		if (headingdeg <= 349) tblOffset = 15;
		else tblOffset = 0;
		}

	tblOffset <<= 7;
	for (inx = 0; inx < 128; inx++) compassBuf[inx] = gCompassTbl[tblOffset++];

	inx = 0;
	for (nrow = 0; nrow < 4; nrow++)  {
		lcd_set_frame_pos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++){
         FrameBuf[128*FramePage + FrameCol+ ncol] = compassBuf[inx++];
         }
		}
	}


#include "bearing.txt"

static void ui_printBearingAnalog(int page, int col,int velkph, int bearingdeg) {
	int  nrow,ncol,inx;
	int tblOffset;
	if (IsGpsHeading && (velkph < 2)) { 
		// if very low velocity, gps heading is uncertain, do not display bearing
		return;
		}
	bearingdeg = bearingdeg % 360;
	if (bearingdeg <= 11) tblOffset = 0;
	else
	if (bearingdeg <= 34) tblOffset = 1;
	else
	if (bearingdeg <= 56) tblOffset = 2;
	else
	if (bearingdeg <= 79) tblOffset = 3;
	else
	if (bearingdeg <= 101) tblOffset = 4;
	else
	if (bearingdeg <= 124) tblOffset = 5;
	else
	if (bearingdeg <= 146) tblOffset = 6;
	else
	if (bearingdeg <= 169) tblOffset = 7;
	else
	if (bearingdeg <= 191) tblOffset = 8;
	else
	if (bearingdeg <= 214) tblOffset = 9;
	else
	if (bearingdeg <= 236) tblOffset = 10;
	else
	if (bearingdeg <= 259) tblOffset = 11;
	else
	if (bearingdeg <= 281) tblOffset = 12;
	else
	if (bearingdeg <= 304) tblOffset = 13;
	else
	if (bearingdeg <= 326) tblOffset = 14;
	else
	if (bearingdeg <= 349) tblOffset = 15;
	else tblOffset = 0;
	tblOffset <<= 5;

	for (inx = 0; inx < 16; inx++) compassBuf[40+inx] = gBearingTbl[tblOffset++];
	for (inx = 0; inx < 16; inx++) compassBuf[72+inx] = gBearingTbl[tblOffset++];

	inx = 0;
	for (nrow = 0; nrow < 4; nrow++)  {
		lcd_set_frame_pos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++) {
         FrameBuf[128*FramePage + FrameCol + ncol] = compassBuf[inx++];
         }
		}
	}


static void ui_printRealTime(int page, int col, int nHrs, int nMin) 	{
	char szBuf[3];
	CLAMP(nHrs,0,23);
	CLAMP(nMin,0,59);
	szBuf[2] = '\0';

	lcd_set_frame_pos(page,col+22);
	lcd_put_char((nMin/10) + 48);
	lcd_put_char((nMin%10) + 48);

	lcd_set_frame_pos(page+1,col+22);
	if (nHrs/12){
		lcd_put_char('p');
		lcd_put_char('m');
		if (nHrs == 12) {
			szBuf[1] = '2';
         szBuf[0] = '1';
			}
		else {
			nHrs = nHrs - 12;
			szBuf[1] = (nHrs%10) + 48;
			nHrs = nHrs/10;
			szBuf[0] = (nHrs ?	nHrs+48 : ' ');
			}	
      	}
   	else
   	if (nHrs < 10)	{
         lcd_put_char('a');
	      lcd_put_char('m');
		   szBuf[1] = nHrs+48;
		   szBuf[0] = ' ';
		   }
   	else {
         lcd_put_char('a');
	      lcd_put_char('m');
		   szBuf[1] = (nHrs%10)+48;
		   szBuf[0] = (nHrs/10)+48;
         }
	lcd_print_sz_lnum(page,col,szBuf);
   }


static void ui_printSpkrStatus(int page, int col, bool bAudioEn) {
	lcd_set_frame_pos(page, col);
	if (bAudioEn) {
      FrameBuf[128*FramePage + FrameCol + 0] = 0x1C;
      FrameBuf[128*FramePage + FrameCol + 1] = 0x14;
      FrameBuf[128*FramePage + FrameCol + 2] = 0x1C;
      FrameBuf[128*FramePage + FrameCol + 3] = 0x22;
      FrameBuf[128*FramePage + FrameCol + 4] = 0x7F;
		}
	else {
		for (int inx = 0; inx < 5; inx++) {
         FrameBuf[128*FramePage + FrameCol + inx] = 0x00;
         }
		}
	}


static void ui_printBluetoothStatus(int page, int col, bool bBluetoothEn) {
	lcd_set_frame_pos(page, col);
	if (bBluetoothEn) {
      FrameBuf[128*FramePage + FrameCol + 0] = 0x22;
      FrameBuf[128*FramePage + FrameCol + 1] = 0x14;
      FrameBuf[128*FramePage + FrameCol + 2] = 0x7F;
      FrameBuf[128*FramePage + FrameCol + 3] = 0x2A;
      FrameBuf[128*FramePage + FrameCol + 4] = 0x14;
		}
	else {
		for (int inx = 0; inx < 5; inx++) {
         FrameBuf[128*FramePage + FrameCol + inx] = 0x00;
         }
		}
	}

static void ui_printBatteryStatus(int page, int col, int bV) {
	lcd_set_frame_pos(page,col);
	FrameBuf[128*FramePage + FrameCol] = 0xFF;
	for (int inx = 0; inx < 14; inx++) {
		FrameBuf[128*FramePage + FrameCol + inx +1] = 0x81;
		}
	FrameBuf[128*FramePage + FrameCol + 15] = 0xE7;
	FrameBuf[128*FramePage + FrameCol + 16] = 0x3C;

	lcd_set_frame_pos(page,col+2);
	int cnt;  // 12 segments, using battery capacity versus voltage data from antoine
	if (bV <= 30) cnt = 0;
	else if (bV == 31) cnt = 1;
	else if (bV < 36) cnt = 2;
	else if (bV < 37) cnt = 5;
	else if (bV < 38) cnt = 8;
	else if (bV < 39)  cnt = 9;
	else if (bV < 40) cnt = 10;
	else if (bV < 41)  cnt = 11;
	else cnt = 12 ;
	for(int inx = 0; inx < cnt; inx++) {
		FrameBuf[128*FramePage + FrameCol + inx] = 0xBD;
		}
	}


static void ui_printSupplyVoltage(int page, int col, int bV) {
	char  szBuf[6];
    sprintf(szBuf,"%d.%dv", bV/10, bV%10);
	lcd_print_sz(page,col,szBuf);
	}


static void ui_printTrackTime(int page, int col, int nHrs, int nMin)	{
	char szBuf[3];
	if (!IsGpsFixStable) {
		lcd_printf(false,page+1,col+22,"??");
		return;
		}
	if (!IsGpsTrackActive) {
		lcd_printf(false,page+1,col+22,"OK");
		return;
		}
	CLAMP(nHrs,0,99);
	CLAMP(nMin,0,59);
	szBuf[2] = '\0';
	szBuf[1] = (nHrs%10) + 48;
	nHrs = nHrs/10;
	szBuf[0] = nHrs ?  nHrs + 48 : ' ';
	lcd_print_sz_lnum(page,col,szBuf);
	lcd_set_frame_pos(page,col+22);
	lcd_put_char((nMin/10) + 48);
	lcd_put_char((nMin%10) + 48);
	}


static void ui_calcTrackElapsedTime(int32_t startmS, int32_t currentmS, int32_t* pHours, int32_t* pMinutes) {
	int32_t elapsedSecs = ((currentmS - startmS)+500)/1000;
	*pHours = elapsedSecs/3600;
	*pMinutes = (elapsedSecs%3600)/60;
	}


static void ui_printRouteSegment(int page, int col, int start, int end) {
   if (start < 0) {
      lcd_printf(false, page,col,"--");
      }
   else {
      lcd_printf(false, page, col,"%2d",start);
	   }
    lcd_printf(false,page, col+24,"%2d",end);
    }


static void ui_alarmWaypointReached() {
   IsFlashDisplayRequired = true;
   for (int cnt = 0; cnt < 3; cnt++) {
	   audio_setFrequency(2000);
	   delayMs(100);
	   audio_setFrequency(0);
	   delayMs(100);
      }
   }


void ui_updateFlightDisplay(NAV_PVT* pn, TRACK* pTrk) {
	// uint32_t marker = cct_setMarker();
	float lat = FLOAT_DEG(pn->nav.latDeg7);
	float lon = FLOAT_DEG(pn->nav.lonDeg7);
	int32_t altm,xaltm;
   	if (opt.misc.altitudeDisplay == ALTITUDE_DISPLAY_GPS){
   		altm = IsGpsFixStable ? (pn->nav.heightMSLmm + 500)/1000 : 9999;
   		xaltm = (int32_t)(KFAltitudeCm/100.0f + 0.5f);
   		}
   	else {
   		altm = (int32_t)(KFAltitudeCm/100.0f + 0.5f);
   		xaltm = IsGpsFixStable ? (pn->nav.heightMSLmm + 500)/1000 : 9999;
   		}

   int32_t dop = (pn->nav.posDOP+50)/100;
   CLAMP(dop, 0, 100);
   if ((!IsGpsFixStable) && (dop < opt.misc.gpsStableDOP)) {
      IsGpsFixStable = true;
      pTrk->startLatdeg = lat;
      pTrk->startLondeg = lon; 
      pTrk->startAltm =  (pn->nav.heightMSLmm + 500)/1000;
      pTrk->maxAltm = pTrk->startAltm;
      pTrk->maxClimbrateCps = -999;
      pTrk->maxSinkrateCps = 999;
      }    

   lcd_clear_frame();
   if (opt.misc.logType == LOGTYPE_IBG) {
      lcd_print_sz(1,74, IsLoggingIBG ? "I" : "i");
      }
   else
   if (opt.misc.logType == LOGTYPE_GPS) {
      lcd_print_sz(1,74, IsGpsTrackActive ? "G" : "g");
      }
   lcd_printf(false,6,104,"%3d*", dop);
   SupplyVoltageMV = adc_supplyVoltageMV();
   ui_printSupplyVoltage(7, 104, (SupplyVoltageMV+50)/100);
   ui_printSpkrStatus(1,56, IsSpeakerEnabled);
   ui_printBluetoothStatus(1,65, IsBluetoothEnabled);
   ui_printAltitude(0,0,altm);
   lcd_print_sz(0,45,opt.misc.altitudeDisplay == ALTITUDE_DISPLAY_GPS ? "g" : "b");
   lcd_print_sz(1,45,"m");
   // secondary altitude display (if primary=baro, secondary=gps, and vice versa)
   if (xaltm == 9999) {
	   lcd_print_sz(0,55,"----");
   	   }
   else {
	   lcd_printf(false,0,55,"%4d",xaltm);
      }
   ui_printClimbRate(2,0,INTEGER_ROUNDUP(DisplayClimbrateCps));
   lcd_print_sz(3,34,"m/s");
   int year,month,day,hour,minute;
   if (pn->nav.numSV > 0) {
      gps_localDateTime(pn,&year,&month,&day,&hour,&minute);
      ui_printRealTime(2,93,hour,minute);
      }

   CompassHeadingDeg = 0;
   if (!IsGpsHeading) { // magnetic compass heading
      CompassHeadingDeg = INTEGER_ROUNDUP(YawDeg);
      CompassHeadingDeg -= (int32_t)opt.misc.magDeclinationdeg;
      CompassHeadingDeg = RANGE_360(CompassHeadingDeg);
      ui_printHeadingAnalog(4,55,0, CompassHeadingDeg);
      }

   if ((pn->nav.numSV > 3) && IsGpsFixStable) {
      pTrk->distanceFromStartm = gps_haversineDistancem(lat, lon, pTrk->startLatdeg, pTrk->startLondeg);
      if ((!IsGpsTrackActive) && (pTrk->distanceFromStartm >= opt.misc.trackStartThresholdm)) {
         IsGpsTrackActive = true;
         pTrk->startTowmS = pn->nav.timeOfWeekmS;
         pTrk->year = year;
         pTrk->month = month;
         pTrk->day = day;
         pTrk->hour = hour;
         pTrk->minute = minute;
         }
      int32_t vn = pn->nav.velNorthmmps;
      int32_t ve = pn->nav.velEastmmps;
	  GpsCourseHeadingDeg = 90 - (int32_t)(atan2((float)vn, (float)ve)*_180_DIV_PI); // course over ground (motion headingdeg)
      GpsCourseHeadingDeg = RANGE_360(GpsCourseHeadingDeg);
      float horzVelmmps = sqrt((float)(vn*vn + ve*ve));
      int32_t horzVelKph = (int32_t)(horzVelmmps*0.0036f + 0.5f);
      ui_printVelocity(4,0,horzVelKph);
      lcd_print_sz(5,34,"kph");
      static float glideRatio = 1.0f;
      if (pn->nav.velDownmmps > 0) { // sinking, display glideratio
         float glideRatioNew = horzVelmmps/(float)pn->nav.velDownmmps;
         glideRatio = (glideRatio*(float)opt.misc.glideRatioIIR + glideRatioNew*(float)(100-opt.misc.glideRatioIIR))/100.0f;
         ui_printGlideRatio(6,0,(int)(glideRatio*10.0f + 0.5f));
         }
      else {
         ui_printGlideRatio(6,0,1000);// climbing, display ++
         }
      lcd_print_sz(7,23,"GR");
      if (IsGpsTrackActive) {
         if (altm > pTrk->maxAltm) pTrk->maxAltm = altm;
         if (DisplayClimbrateCps > pTrk->maxClimbrateCps) pTrk->maxClimbrateCps = DisplayClimbrateCps;
         if (DisplayClimbrateCps < pTrk->maxSinkrateCps) pTrk->maxSinkrateCps = DisplayClimbrateCps;
         ui_calcTrackElapsedTime(pTrk->startTowmS, pn->nav.timeOfWeekmS, &pTrk->elapsedHours, &pTrk->elapsedMinutes);
         }
      ui_printTrackTime(4,93,pTrk->elapsedHours,pTrk->elapsedMinutes);
      int32_t bearingDeg, distancem;
      //int32_t gpsCourseHeadingDeg = pn->nav.headingMotionDeg5/100000; // this gives junk readings
      if (IsGpsHeading) {
         ui_printHeadingAnalog(4,55,horzVelKph, GpsCourseHeadingDeg);
         }
      if (IsRouteActive) { // show bearingdeg relative to course/compass headingdeg and distance to next waypoint
         if (pRoute->nextWptInx >= pRoute->numWpts) {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg) - (IsGpsHeading ?  GpsCourseHeadingDeg : CompassHeadingDeg);
            bearingDeg = RANGE_360(bearingDeg);
            distancem = gps_haversineDistancem(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg);
            ui_printRouteSegment(3, 52, pRoute->numWpts-1, pRoute->numWpts-1);
            }
         else {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->nextWptInx].latdeg, pRoute->wpt[pRoute->nextWptInx].londeg) - (IsGpsHeading ?  GpsCourseHeadingDeg : CompassHeadingDeg);
            bearingDeg = RANGE_360(bearingDeg);
            distancem = gps_haversineDistancem(lat, lon, pRoute->wpt[pRoute->nextWptInx].latdeg, pRoute->wpt[pRoute->nextWptInx].londeg);
            ui_printRouteSegment(3, 52, pRoute->nextWptInx-1, pRoute->nextWptInx);
            if (distancem < pRoute->wpt[pRoute->nextWptInx].radiusm) {
               ui_alarmWaypointReached();
               pRoute->nextWptInx++;
               }
            }
         }
      else { // no route, show bearingdeg relative to course/compassheading and distance to start
         bearingDeg = gps_bearingDeg(lat, lon, pTrk->startLatdeg, pTrk->startLondeg) - (IsGpsHeading ? GpsCourseHeadingDeg : CompassHeadingDeg);
         bearingDeg = RANGE_360(bearingDeg);
         distancem = pTrk->distanceFromStartm;
         }
      ui_printBearingAnalog(4,55, horzVelKph, bearingDeg);     
      ui_printDistance(0, 82, distancem);
      lcd_print_sz(1,116,"km");
      }
   
   // invert (white on black) frame, used to acknowledge button press
   if (IsFlashDisplayRequired) {
      IsFlashDisplayRequired = false;
      lcd_invert_frame();
      }
   lcd_send_frame();
//   uint32_t eus = cct_elapsedUs(marker);
  // ESP_LOGI(TAG,"updateFlightDisplay = %dus", eus);
   }


int ui_saveFlightLogSummary(NAV_PVT* pn, TRACK* pTrk) {
	char szbuf[30];
	char szEntry[110];
	ssize_t nwrote;
	int len;
	File fd = LITTLEFS.open("/flightlog.txt", FILE_READ);
	// if flight log does not exist, create it with a comment header
	if (!fd) {
		fd = LITTLEFS.open("/flightlog.txt", FILE_WRITE);
		sprintf(szEntry,"Year Month Day sHour sMin durHour durMin sLat sLon sAlt eLat eLon eAlt maxAlt maxClimb maxSink\r\n");
		nwrote = fd.print(szEntry);
		if (nwrote != len) {
			ESP_LOGE(TAG,"Error writing comment header to flightlog.txt");
			fd.close();
			return -1;
			}
		}  
	fd.close();

	// open in append mode
	fd = LITTLEFS.open("/flightlog.txt", FILE_APPEND);
	if (!fd) {
		ESP_LOGE(TAG,"Error opening flightlog.txt in append mode");
		return -2;
		}

	sprintf(szEntry,"%4d %2d %2d %2d %2d ", pTrk->year, pTrk->month, pTrk->day, pTrk->hour, pTrk->minute);
	sprintf(szbuf,"%2d %2d ", pTrk->elapsedHours, pTrk->elapsedMinutes);
	strcat(szEntry, szbuf);

	sprintf(szbuf,"%f %f %4d ", pTrk->startLatdeg, pTrk->startLondeg, (int)(pTrk->startAltm+0.5f));
	strcat(szEntry, szbuf);

	sprintf(szbuf,"%f %f %4d ", FLOAT_DEG(pn->nav.latDeg7), FLOAT_DEG(pn->nav.lonDeg7), (pn->nav.heightMSLmm+500)/1000);
	strcat(szEntry, szbuf);

	sprintf(szbuf,"%4d ", (int)(pTrk->maxAltm+0.5f));
	strcat(szEntry, szbuf);

	sprintf(szbuf,"%.1f %.1f\r\n", pTrk->maxClimbrateCps/100.0f, pTrk->maxSinkrateCps/100.0f);
	strcat(szEntry, szbuf);

	nwrote = fd.print(szEntry);
	if (nwrote != strlen(szEntry)) {
		ESP_LOGE(TAG,"Error appending entry to flightlog.txt");
		fd.close();
		return -1;
		}
	fd.close();
   	return 0;
   	}


void ui_screenInit() {
	ParChanged = 0;
	ScreenParOffset = 0;
	ParDisplaySel = 0;
	ParSel = 0;
	}


void ui_displayOptions(void) {
  int row;
  lcd_clear();
  for (row = 0; row < 8; row++) {
    if ((ScreenParOffset + row) == ParSel) {
       lcd_printf(false, row, 0, ParDisplaySel ? "*" : "o");
       }
    else {
       lcd_printf(false, row, 0, " ");
       }
    }

  row = 0;
  if (ScreenParOffset < 1) {
    lcd_printf(false, row, 7, "Climb threshold  %3d", opt.vario.climbThresholdCps); row++;
    }
  if (ScreenParOffset < 2) {
    lcd_printf(false, row, 7, "Zero threshold   %3d", opt.vario.zeroThresholdCps); row++;
    }
  if (ScreenParOffset < 3){
    lcd_printf(false, row, 7, "Sink threshold  %4d", opt.vario.sinkThresholdCps); row++;
    }
  if (ScreenParOffset < 4) {
     lcd_printf(false, row, 7, "XOver threshold %4d", opt.vario.crossoverCps); row++;
     }   
  if (ScreenParOffset < 5) {
     lcd_printf(false, row, 7, "Vario display IIR %2d", opt.vario.varioDisplayIIR); row++;
     }   
  if (ScreenParOffset < 6) {
     lcd_printf(false, row, 7, "Accel variance   %3d", opt.kf.accelVariance); row++;
     }
  if (ScreenParOffset < 7) {
     lcd_printf(false, row, 7, "Zmeas variance   %3d", opt.kf.zMeasVariance); row++;
     }
  if (ScreenParOffset < 8) {
     lcd_printf(false, row, 7, "UTC offset      %4d",  opt.misc.utcOffsetMins); row++;
     }
  if (ScreenParOffset < 9) {
     lcd_printf(false, row, 7, "Backlight secs    %2d",  opt.misc.backlitSecs); row++;
     }
  if (ScreenParOffset < 10) {
     lcd_printf(false, row, 7, "Track Threshold  %3d",  opt.misc.trackStartThresholdm); row++;
     }
  if (ScreenParOffset < 11) {
     lcd_printf(false, row, 7, "Track Interval    %2d",  opt.misc.trackIntervalSecs); row++;
     }
  if (ScreenParOffset < 12) {
     lcd_printf(false, row, 7, "GlideRatio IIR    %2d",  opt.misc.glideRatioIIR); row++;
     }
  if (ScreenParOffset < 13) {
     lcd_printf(false, row, 7, "GPS Stable DOP    %2d",  opt.misc.gpsStableDOP); row++;
     }
  if (ScreenParOffset < 14) {
     lcd_printf(false, row, 7, "Gyro Offset Max  %3d",  opt.misc.gyroOffsetLimit1000DPS); row++;
     }
  if (ScreenParOffset < 15) {
     lcd_printf(false, row, 7, "Mag Declination  %3d",  opt.misc.magDeclinationdeg); row++;
     }
  if (ScreenParOffset < 16) {
     lcd_printf(false, row, 7, "Speaker Volume     %1d",  opt.misc.speakerVolume); row++;
     }
  if (ScreenParOffset < 17) {
     lcd_printf(false, row, 7, "Log Type        %s",  szLogType[opt.misc.logType]); row++;
     }
  if (ScreenParOffset < 18) {
     lcd_printf(false, row, 7, "Waypt Radius   %5d",  opt.misc.waypointRadiusm); row++;
     }
  if (ScreenParOffset < 19) {
     lcd_printf(false, row, 7, "Alt display     %s",  szAltDisplayType[opt.misc.altitudeDisplay]); row++;
     }
  if (ScreenParOffset < 20) {
     lcd_printf(false, row, 7, "BtMsg Type       %s",  szBtMsgType[opt.misc.btMsgType]); row++;
     }
  if (ScreenParOffset < 21) {
     lcd_printf(false, row, 7, "BtMsg Freq        %2d",  opt.misc.btMsgFreqHz); row++;
     }
  if (ScreenParOffset < 22) {
     lcd_printf(false, row, 7, "LCD Contrast      %2d",  opt.misc.lcdContrast); row++;
     }
  lcd_send_frame();
  }

#define OPT_IDLE_COUNT 320 // this is ~8 seconds of inactivity with 25mS debounce interval

bool ui_optionsEventHandler(void)  {
	static int countDown = OPT_IDLE_COUNT;
	if (Btn0Pressed || (countDown <= 0)) {
		btn_clear();
		if (ParChanged) {
			ParChanged = 0;
			lcd_clear_frame();
			lcd_printlnf(true, 0, "Saving options");
			opt_save();
			delayMs(1000);
			}
		return true;
		}
  
	if (BtnMPressed)	{
		btn_clear();
		countDown = OPT_IDLE_COUNT;
		ParDisplaySel = !ParDisplaySel;
		ui_displayOptions();
		return false;
		}
	
	if (BtnLPressed) {
		btn_clear();
		countDown = OPT_IDLE_COUNT;
		if (ParDisplaySel) {
			ParChanged = 1;
			switch (ParSel) {
				case SEL_CLIMB_THRESHOLD : if (opt.vario.climbThresholdCps >= VARIO_CLIMB_THRESHOLD_CPS_MIN+5) opt.vario.climbThresholdCps -= 5;
				default :
				break;
				 	
				case SEL_ZERO_THRESHOLD : if (opt.vario.zeroThresholdCps >= VARIO_ZERO_THRESHOLD_CPS_MIN+5) opt.vario.zeroThresholdCps -= 5;
				break;
	
				case SEL_SINK_THRESHOLD : if (opt.vario.sinkThresholdCps >= VARIO_SINK_THRESHOLD_CPS_MIN+5) opt.vario.sinkThresholdCps -= 5;
				break;

				case SEL_CROSSOVER_THRESHOLD : if (opt.vario.crossoverCps >= VARIO_CROSSOVER_CPS_MIN+10) opt.vario.crossoverCps -= 10;
				break;

				case SEL_VARIO_DISPLAY_IIR : if (opt.vario.varioDisplayIIR > VARIO_DISPLAY_IIR_MIN) opt.vario.varioDisplayIIR--;
				break;

				case SEL_ACCEL_VAR : if (opt.kf.accelVariance >= KF_ACCEL_VARIANCE_MIN+5) opt.kf.accelVariance -= 5;
				break;

				case SEL_ZMEAS_VAR : if (opt.kf.zMeasVariance >= KF_ZMEAS_VARIANCE_MIN+5) opt.kf.zMeasVariance -= 5;
				break;

				case SEL_UTC_OFFSET : if (opt.misc.utcOffsetMins >= UTC_OFFSET_MINS_MIN+15) opt.misc.utcOffsetMins -= 15;
				break;

				case SEL_BKLIGHT_SECS : if (opt.misc.backlitSecs >= BACKLIT_SECS_MIN+5) opt.misc.backlitSecs -= 5;
				break;

				case SEL_TRACK_THRESHOLD : if (opt.misc.trackStartThresholdm >= TRACK_START_THRESHOLD_M_MIN+5 ) opt.misc.trackStartThresholdm -= 5;
				break;

				case SEL_TRACK_INTERVAL : if (opt.misc.trackIntervalSecs > TRACK_INTERVAL_SECS_MIN) opt.misc.trackIntervalSecs--;
				break;

				case SEL_GLIDE_IIR : if (opt.misc.glideRatioIIR > GLIDE_RATIO_IIR_MIN) opt.misc.glideRatioIIR--;
				break;

				case SEL_GPS_STABLE_DOP : if (opt.misc.gpsStableDOP > GPS_STABLE_DOP_MIN) opt.misc.gpsStableDOP--;
				break;

				case SEL_GYRO_OFFSET_MAX : if (opt.misc.gyroOffsetLimit1000DPS >= GYRO_OFFSET_LIMIT_1000DPS_MIN+10) opt.misc.gyroOffsetLimit1000DPS -= 10;
				break;

				case SEL_MAG_DECLINATION : if (opt.misc.magDeclinationdeg > MAG_DECLINATION_DEG_MIN) opt.misc.magDeclinationdeg--;
				break;

				case SEL_SPKR_VOL : if (opt.misc.speakerVolume > SPEAKER_VOLUME_MIN ) opt.misc.speakerVolume--;
				break;

				case SEL_LOG_TYPE : if (opt.misc.logType > LOGTYPE_NONE ) opt.misc.logType--;
				break;

				case SEL_WPT_RADIUS : if (opt.misc.waypointRadiusm >= WAYPOINT_RADIUS_M_MIN+10  ) opt.misc.waypointRadiusm -= 10;
				break;

				case SEL_ALTITUDE_DISPLAY : if (opt.misc.altitudeDisplay > ALTITUDE_DISPLAY_GPS  ) opt.misc.altitudeDisplay--;
				break;

				case SEL_BTMSG_TYPE : if (opt.misc.btMsgType > BT_MSG_LK8EX1  ) opt.misc.btMsgType--;
				break;

				case SEL_BTMSG_FREQ : if (opt.misc.btMsgFreqHz > BT_MSG_FREQ_HZ_MIN  ) opt.misc.btMsgFreqHz--;
				break;

				case SEL_LCD_CONTRAST : if (opt.misc.lcdContrast > LCD_CONTRAST_MIN  ) {
					opt.misc.lcdContrast--;
				    lcd_send_cmd(CMD_SET_VOLUME_FIRST);
				    lcd_send_cmd(CMD_SET_VOLUME_SECOND | (opt.misc.lcdContrast & 0x3f));
					}
				break;
 	      	  	}
		  	}
		else { // ParDisplaySel == 0
			if (ParSel > 0) ParSel--;
			if (ScreenParOffset > ParSel) {
				ScreenParOffset = ParSel;
            	}
		   	 }
      ui_displayOptions();
      return false;
      }
   
	if (BtnRPressed) {
		btn_clear();
		countDown = OPT_IDLE_COUNT;
		if (ParDisplaySel) {
			ParChanged = 1;
			switch (ParSel) {

				case SEL_CLIMB_THRESHOLD : if (opt.vario.climbThresholdCps <= VARIO_CLIMB_THRESHOLD_CPS_MAX-5) opt.vario.climbThresholdCps += 5;
				default :
				break;
				 	
				case SEL_ZERO_THRESHOLD : if (opt.vario.zeroThresholdCps <= VARIO_ZERO_THRESHOLD_CPS_MAX-5) opt.vario.zeroThresholdCps += 5;
				break;
	
				case SEL_SINK_THRESHOLD : if (opt.vario.sinkThresholdCps <= VARIO_SINK_THRESHOLD_CPS_MAX-5) opt.vario.sinkThresholdCps += 5;
				break;

				case SEL_CROSSOVER_THRESHOLD : if (opt.vario.crossoverCps <= VARIO_CROSSOVER_CPS_MAX-10) opt.vario.crossoverCps += 10;
				break;

				case SEL_VARIO_DISPLAY_IIR : if (opt.vario.varioDisplayIIR < VARIO_DISPLAY_IIR_MAX) opt.vario.varioDisplayIIR++;
				break;

				case SEL_ACCEL_VAR : if (opt.kf.accelVariance <= KF_ACCEL_VARIANCE_MAX-5) opt.kf.accelVariance += 5;
				break;

				case SEL_ZMEAS_VAR : if (opt.kf.zMeasVariance <= KF_ZMEAS_VARIANCE_MAX-5) opt.kf.zMeasVariance += 5;
				break;

				case SEL_UTC_OFFSET : if (opt.misc.utcOffsetMins <= UTC_OFFSET_MINS_MAX-15) opt.misc.utcOffsetMins += 15;
				break;

				case SEL_BKLIGHT_SECS : if (opt.misc.backlitSecs < BACKLIT_SECS_MAX) opt.misc.backlitSecs++;
				break;

				case SEL_TRACK_THRESHOLD : if (opt.misc.trackStartThresholdm <= TRACK_START_THRESHOLD_M_MAX-5 ) opt.misc.trackStartThresholdm += 5;
				break;

				case SEL_TRACK_INTERVAL : if (opt.misc.trackIntervalSecs < TRACK_INTERVAL_SECS_MAX) opt.misc.trackIntervalSecs++;
				break;

				case SEL_GLIDE_IIR : if (opt.misc.glideRatioIIR < GLIDE_RATIO_IIR_MAX) opt.misc.glideRatioIIR++;
				break;

				case SEL_GPS_STABLE_DOP : if (opt.misc.gpsStableDOP < GPS_STABLE_DOP_MAX) opt.misc.gpsStableDOP++;
				break;

				case SEL_GYRO_OFFSET_MAX : if (opt.misc.gyroOffsetLimit1000DPS <= GYRO_OFFSET_LIMIT_1000DPS_MAX-10) opt.misc.gyroOffsetLimit1000DPS += 10;
				break;

				case SEL_MAG_DECLINATION : if (opt.misc.magDeclinationdeg < MAG_DECLINATION_DEG_MAX) opt.misc.magDeclinationdeg++;
				break;

				case SEL_SPKR_VOL : if (opt.misc.speakerVolume < SPEAKER_VOLUME_MAX ) opt.misc.speakerVolume++;
				break;

				case SEL_LOG_TYPE : if (opt.misc.logType < LOGTYPE_IBG ) opt.misc.logType++;
				break;

				case SEL_WPT_RADIUS : if (opt.misc.waypointRadiusm <= WAYPOINT_RADIUS_M_MAX-10  ) opt.misc.waypointRadiusm += 10;
				break;

				case SEL_ALTITUDE_DISPLAY : if (opt.misc.altitudeDisplay < ALTITUDE_DISPLAY_BARO) opt.misc.altitudeDisplay++;
				break;

				case SEL_BTMSG_TYPE : if (opt.misc.btMsgType < BT_MSG_XCTRC  ) opt.misc.btMsgType++;
				break;

				case SEL_BTMSG_FREQ : if (opt.misc.btMsgFreqHz < BT_MSG_FREQ_HZ_MAX  ) opt.misc.btMsgFreqHz++;
				break;

				case SEL_LCD_CONTRAST : if (opt.misc.lcdContrast < LCD_CONTRAST_MAX  ) {
					opt.misc.lcdContrast++;
				    lcd_send_cmd(CMD_SET_VOLUME_FIRST);
				    lcd_send_cmd(CMD_SET_VOLUME_SECOND | (opt.misc.lcdContrast & 0x3f));
					}
				break;
		      	  }
	      	  }
		else { // ParDisplaySel == 0
			if (ParSel <  SEL_LCD_CONTRAST) ParSel++;
			if (ScreenParOffset < (ParSel-7)) {
				ScreenParOffset = ParSel-7;
            	}
         	 }
		ui_displayOptions();
		return false;
  	   	}
	countDown--;
	return false;
   	}

