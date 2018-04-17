#include "common.h"
#include "config.h"
#include "lcd7565.h"
#include "btn.h"
#include "ui.h"
#include "myadc.h"
#include "gps.h"
#include "ms5611.h"
#include "options.h"
#include "route.h"
#include "cct.h"
#include "audio.h"
#include <math.h>

#define TAG "ui"

bool IsRouteActive = false;
bool IsSpeakerEnabled  = true;
bool IsGpsFixStable = false;
bool IsTrackActive = false;
bool IsLcdBkltEnabled = false;
bool IsLogging = false;
bool IsFlashDisplayRequired = false;
bool IsGpsHeading = true;
bool EndTrack = false;

static int ParChanged = 0;
static int ScreenParOffset = 0;
static int ParDisplaySel = 0;
static int ParSel = 0;

void ui_printLatitude(int page, int col, int32_t nLat) {
	char szBuf[12];
	int32_t  nL;
	int inx;
	nL = ABS(nLat)/100;
	sprintf(szBuf,"%c%d", (nLat >= 0 ? 'N' : 'S'), nL/100000);
	lcd_setFramePos(page,col);
	lcd_printSzLNum(page,col,szBuf);
	sprintf(szBuf,"%05d", nL%100000);
	inx = 0;
	while (szBuf[inx]) {
		lcd_putChar(szBuf[inx++]);
      }
   }


void ui_printLongitude(int page, int col, int32_t nLon) {
	char  szBuf[12];
   int32_t  nL;
   int inx;
   nL = ABS(nLon)/100;
   sprintf(szBuf,"%c%d", (nLon >= 0 ? 'E' : 'W'), nL/100000);
	lcd_setFramePos(page,col);
	lcd_printSzLNum(page,col,szBuf);
   sprintf(szBuf,"%05d", nL%100000);
   inx = 0;
	while (szBuf[inx]) {
		lcd_putChar(szBuf[inx++]);
      }
   }


void ui_printAltitude(int page, int col, int32_t altm) {
	char szBuf[5];
   CLAMP(altm,MIN_ALTITUDE_M,MAX_ALTITUDE_M);
   sprintf(szBuf,"%4d", altm);
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printDistance(int page, int col, int distanceM) {
	char szBuf[4];
   CLAMP(distanceM,MIN_DISTANCE_M,MAX_DISTANCE_M);
   distanceM = (distanceM + 5)/10;
	lcd_setFramePos(page,col+40);
	lcd_putChar(48+ (distanceM%10));
	distanceM = distanceM/10;
	lcd_setFramePos(page,col+33);
	lcd_putChar(48+(distanceM%10));
	distanceM = distanceM/10;
   sprintf(szBuf,"%3d", distanceM);
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printClimbRate(int page, int col, int nCps) {
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
	lcd_setFramePos(page,col+33);
   if (sign == ' ') {
		lcd_putChar('0');
	  	szBuf[2] = '0';
     	}
   else {
   	dec = nCps/10;
   	frac = nCps%10;
		lcd_putChar(frac+48);
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
	lcd_printSzLNum(page,col,szBuf);
  	}


void ui_printVelocity(int page, int col, int nVel) {
	char  szBuf[4];
   CLAMP(nVel, 0, MAX_VELOCITY_KPH);
   sprintf(szBuf, "%3d", nVel);
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printGlideRatio(int page, int col, int nGr) {
	char  szBuf[3];
	szBuf[2] = '\0';
	if (nGr > 999) {
	   lcd_setFramePos(page,col+24);
	   lcd_putChar(' ');
		szBuf[1] = '+';
		szBuf[0] = ' ';
		}
	else{
	   lcd_setFramePos(page,col+24);
	   lcd_putChar(48+(nGr%10));
	   nGr = nGr/10;
		szBuf[1] = (nGr%10) + 48;
		nGr = nGr/10;
		szBuf[0] = nGr ? (nGr+48) : ' ';
	   }
	lcd_printSzLNum(page,col,szBuf);
	}


#include "compass.txt"

static uint8_t compassBuf[128];

void ui_printHeadingAnalog(int page, int col, int velkph, int headingdeg) {
	int nrow,ncol,inx;
	int tblOffset = 0;
	lcd_setFramePos(page - 1, col+13);

   if (IsGpsHeading) {
      FrameBuf[128*FramePage + FrameCol+1] = 0x3C;
      FrameBuf[128*FramePage + FrameCol+2] = 0x7C;
      FrameBuf[128*FramePage + FrameCol+3] = 0x3C;  
      // if very low velocity, gps heading is uncertain, do not display
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
		lcd_setFramePos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++){
         FrameBuf[128*FramePage + FrameCol+ ncol] = compassBuf[inx++];
         }
		}
	}


#include "bearing.txt"

void ui_printBearingAnalog(int page, int col,int velkph, int bearingdeg) {
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
		lcd_setFramePos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++) {
         FrameBuf[128*FramePage + FrameCol + ncol] = compassBuf[inx++];
         }
		}
	}


void ui_printRealTime(int page, int col, int nHrs, int nMin) 	{
	char szBuf[3];
   CLAMP(nHrs,0,23);
   CLAMP(nMin,0,59);
	szBuf[2] = '\0';

	lcd_setFramePos(page,col+22);
	lcd_putChar((nMin/10) + 48);
	lcd_putChar((nMin%10) + 48);

	lcd_setFramePos(page+1,col+22);
   if (nHrs/12){
      lcd_putChar('p');
	   lcd_putChar('m');
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
         lcd_putChar('a');
	      lcd_putChar('m');
		   szBuf[1] = nHrs+48;
		   szBuf[0] = ' ';
		   }
   	else {
         lcd_putChar('a');
	      lcd_putChar('m');
		   szBuf[1] = (nHrs%10)+48;
		   szBuf[0] = (nHrs/10)+48;
         }
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printSpkrStatus(int page, int col, int bAudioEn) {
	lcd_setFramePos(page, col);
	if (bAudioEn) {
      FrameBuf[128*FramePage + FrameCol + 0] = 0x3C;
      FrameBuf[128*FramePage + FrameCol + 1] = 0x24;
      FrameBuf[128*FramePage + FrameCol + 2] = 0x3C;
      FrameBuf[128*FramePage + FrameCol + 3] = 0x42;
      FrameBuf[128*FramePage + FrameCol + 4] = 0x81;
      FrameBuf[128*FramePage + FrameCol + 5] = 0xFF;
		}
	else {
		for (int inx = 0; inx < 6; inx++) {
         FrameBuf[128*FramePage + FrameCol + inx] = 0x00;
         }
		}
	}


void ui_printBatteryStatus(int page, int col, int bV) {
	lcd_setFramePos(page,col);
   FrameBuf[128*FramePage + FrameCol] = 0xFF;
   for (int inx = 0; inx < 14; inx++) {
      FrameBuf[128*FramePage + FrameCol + inx +1] = 0x81;
      }
   FrameBuf[128*FramePage + FrameCol + 15] = 0xE7;
   FrameBuf[128*FramePage + FrameCol + 16] = 0x3C;

	lcd_setFramePos(page,col+2);
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



void ui_printTrackTime(int page, int col, int nHrs, int nMin)	{
	char szBuf[3];
	if (!IsGpsFixStable) {
		lcd_printf(false,page+1,col+22,"??");
		return;
		}
	if (!IsTrackActive) {
		lcd_printf(false,page+1,col+22,"OK");
		return;
		}
   CLAMP(nHrs,0,99);
   CLAMP(nMin,0,59);
	szBuf[2] = '\0';
	szBuf[1] = (nHrs%10) + 48;
   nHrs = nHrs/10;
	szBuf[0] = nHrs ?  nHrs + 48 : ' ';
	lcd_printSzLNum(page,col,szBuf);
	lcd_setFramePos(page,col+22);
	lcd_putChar((nMin/10) + 48);
	lcd_putChar((nMin%10) + 48);
   }


void ui_printPosDOP(int page, int col, int dop) {
   lcd_printf(false,page,col,"%3d", dop);
   }


void ui_calcTrackElapsedTime(int32_t startmS, int32_t currentmS, int32_t* pHours, int32_t* pMinutes) {
   int32_t elapsedSecs = ((currentmS - startmS)+500)/1000;
   *pHours = elapsedSecs/3600;
   *pMinutes = (elapsedSecs%3600)/60;
   }


void ui_printRouteSegment(int page, int col, int start, int end) {
   if (start < 0) {
      lcd_printf(false, page,col,"--");
      }
   else {
      lcd_printf(false, page, col,"%2d",start);
	   }
    lcd_printf(false,page, col+24,"%2d",end);
    }


void ui_alarmWaypointReached() {
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
   int32_t altm = IsGpsFixStable ? (pn->nav.heightMSLmm + 500)/1000 : (int32_t)(KFAltitudeCm/100.0f + 0.5f);
   int32_t dop = (pn->nav.posDOP+50)/100;
   if ((!IsGpsFixStable) && (dop < opt.misc.gpsStableDOP)) {
      IsGpsFixStable = true;
      pTrk->startLatdeg = lat;
      pTrk->startLondeg = lon; 
      pTrk->startAltm =  (pn->nav.heightMSLmm + 500)/1000;
      pTrk->maxAltm = pTrk->startAltm;
      pTrk->maxClimbrateCps = -999;
      pTrk->maxSinkrateCps = 999;
      }    

   lcd_clearFrame();
   if (opt.misc.logType == LOGTYPE_IBG) {
      lcd_printSz(0,60, IsLogging ? "IBG" : "ibg");
      }
   else
   if (opt.misc.logType == LOGTYPE_GPS) {
      lcd_printSz(0,60, IsTrackActive ? "GPS" : "gps");
      }
   ui_printPosDOP(6,110, dop);
   int batv = (int)(adc_batteryVoltage()+50)/100;
   ui_printBatteryStatus(7, 111, batv);
   ui_printSpkrStatus(7,100, IsSpeakerEnabled);
   ui_printAltitude(0,0,altm);
   lcd_printSz(1,45,"m");
   ui_printClimbRate(2,0,INTEGER_ROUNDUP(IIRClimbrateCps));
   lcd_printSz(3,34,"ms");

   int year,month,day,hour,minute;
   if (pn->nav.numSV > 0) {
      gps_localDateTime(pn,&year,&month,&day,&hour,&minute);
      ui_printRealTime(2,93,hour,minute);
      }

   int32_t compassHeadingDeg = 0;
   if (!IsGpsHeading) { // magnetic compass heading
      compassHeadingDeg = INTEGER_ROUNDUP(YawDeg);
      compassHeadingDeg -= (int32_t)opt.misc.magDeclinationdeg; 
      compassHeadingDeg = RANGE_360(compassHeadingDeg);
      ui_printHeadingAnalog(4,55,0, compassHeadingDeg);
      }

   if ((pn->nav.numSV > 3) && IsGpsFixStable) {
      pTrk->distanceFromStartm = gps_haversineDistancem(lat, lon, pTrk->startLatdeg, pTrk->startLondeg);
      if ((!IsTrackActive) && (pTrk->distanceFromStartm >= opt.misc.trackStartThresholdm)) {
         IsTrackActive = true;
         pTrk->startTowmS = pn->nav.timeOfWeekmS;
         pTrk->year = year;
         pTrk->month = month;
         pTrk->day = day;
         pTrk->hour = hour;
         pTrk->minute = minute;
         }
      int32_t vn = pn->nav.velNorthmmps;
      int32_t ve = pn->nav.velEastmmps;
      int32_t gpsCourseHeadingDeg = 90 - (int32_t)(atan2((float)vn, (float)ve)*_180_DIV_PI); // course over ground (motion headingdeg)
      gpsCourseHeadingDeg = RANGE_360(gpsCourseHeadingDeg);
      float horzVelmmps = sqrt((float)(vn*vn + ve*ve));
      int32_t horzVelKph = (int32_t)(horzVelmmps*0.0036f + 0.5f);
      ui_printVelocity(4,0,horzVelKph);
      lcd_printSz(5,34,"kh");
      static float glideRatio = 1.0f;
      if (pn->nav.velDownmmps > 0) { // sinking, display glideratio
         float glideRatioNew = horzVelmmps/(float)pn->nav.velDownmmps;
         glideRatio = (glideRatio*(float)opt.misc.glideRatioIIR + glideRatioNew*(float)(100-opt.misc.glideRatioIIR))/100.0f;
         ui_printGlideRatio(6,0,(int)(glideRatio*10.0f + 0.5f));
         }
      else {
         ui_printGlideRatio(6,0,1000);// climbing, display ++
         }
      lcd_printSz(7,23,"gr");
      if (IsTrackActive) {
         if (altm > pTrk->maxAltm) pTrk->maxAltm = altm;
         if (IIRClimbrateCps > pTrk->maxClimbrateCps) pTrk->maxClimbrateCps = IIRClimbrateCps;
         if (IIRClimbrateCps < pTrk->maxSinkrateCps) pTrk->maxSinkrateCps = IIRClimbrateCps;
         ui_calcTrackElapsedTime(pTrk->startTowmS, pn->nav.timeOfWeekmS, &pTrk->elapsedHours, &pTrk->elapsedMinutes);
         }
      ui_printTrackTime(4,93,pTrk->elapsedHours,pTrk->elapsedMinutes);
      int32_t bearingDeg, distancem;
      //int32_t gpsCourseHeadingDeg = pn->nav.headingMotionDeg5/100000; // this gives junk readings
      if (IsGpsHeading) {
         ui_printHeadingAnalog(4,55,horzVelKph, gpsCourseHeadingDeg);
         }
      if (IsRouteActive) { // show bearingdeg relative to course/compass headingdeg and distance to next waypoint
         if (pRoute->nextWptInx >= pRoute->numWpts) {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg) - (IsGpsHeading ?  gpsCourseHeadingDeg : compassHeadingDeg);
            bearingDeg = RANGE_360(bearingDeg);
            distancem = gps_haversineDistancem(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg);
            ui_printRouteSegment(3, 52, pRoute->numWpts-1, pRoute->numWpts-1);
            }
         else {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->nextWptInx].latdeg, pRoute->wpt[pRoute->nextWptInx].londeg) - (IsGpsHeading ?  gpsCourseHeadingDeg : compassHeadingDeg);
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
         bearingDeg = gps_bearingDeg(lat, lon, pTrk->startLatdeg, pTrk->startLondeg) - (IsGpsHeading ? gpsCourseHeadingDeg : compassHeadingDeg);
         bearingDeg = RANGE_360(bearingDeg);
         distancem = pTrk->distanceFromStartm;
         }
      ui_printBearingAnalog(4,55, horzVelKph, bearingDeg);
      ui_printDistance(0, 82, distancem);
      lcd_printSz(1,116,"km");
      }
   

   // invert (white on black) frame, used to acknowledge button press
   if (IsFlashDisplayRequired) {
      IsFlashDisplayRequired = false;
      lcd_invertFrame();
      }
   lcd_sendFrame();
//   uint32_t eus = cct_elapsedUs(marker);
  // ESP_LOGI(TAG,"updateFlightDisplay = %dus", eus);
   }


int ui_saveLog(NAV_PVT* pn, TRACK* pTrk) {
   FILE *fdwr;
   char buf[80];
   ssize_t nwrote;

   sprintf(buf,"/spiffs/%04d%02d%02d_%02d%02d.txt", pTrk->year, pTrk->month, pTrk->day, pTrk->hour, pTrk->minute);
   fdwr = fopen(buf, "wb");
   if (fdwr == NULL) return -1;

   sprintf(buf,"Start %04d/%02d/%02d %02d:%02d\r\n", pTrk->year, pTrk->month, pTrk->day, pTrk->hour, pTrk->minute);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Duration %02d:%02d\r\n", pTrk->elapsedHours, pTrk->elapsedMinutes);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Start Latitude %f Longitude %f Altitude %dm\r\n", pTrk->startLatdeg, pTrk->startLondeg, (int)(pTrk->startAltm+0.5f));
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"End Latitude %f Longitude %f Altitude %dm\r\n", FLOAT_DEG(pn->nav.latDeg7), FLOAT_DEG(pn->nav.lonDeg7), (pn->nav.heightMSLmm+500)/1000);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Max Altitude  %dm\r\n", (int)(pTrk->maxAltm+0.5f));
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Max Climbrate  +%.1fm/s\r\n", pTrk->maxClimbrateCps/100.0f);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Max Sinkrate  %.1fm/s\r\n", pTrk->maxSinkrateCps/100.0f);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   fclose(fdwr);
   return 0;
   }


void ui_screenInit() {
   ParChanged = 0;
   ScreenParOffset = 0;
   ParDisplaySel = 0;
   ParSel = 0;
   }

char szLogType[3][5] = {"NONE", " GPS", " IBG"};

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
     lcd_printf(false, row, 7, "Accel variance    %2d", opt.kf.accelVariance); row++;
     }
  if (ScreenParOffset < 6) {
     lcd_printf(false, row, 7, "Zmeas variance   %3d", opt.kf.zMeasVariance); row++;
     }
  if (ScreenParOffset < 7) {
     lcd_printf(false, row, 7, "UTC offset     %4d",  opt.misc.utcOffsetMins); row++;
     }
  if (ScreenParOffset < 8) {
     lcd_printf(false, row, 7, "Backlight secs    %2d",  opt.misc.backlitSecs); row++;
     }
  if (ScreenParOffset < 9) {
     lcd_printf(false, row, 7, "Track Threshold  %3d",  opt.misc.trackStartThresholdm); row++;
     }
  if (ScreenParOffset < 10) {
     lcd_printf(false, row, 7, "Track Interval    %2d",  opt.misc.trackIntervalSecs); row++;
     }
  if (ScreenParOffset < 11) {
     lcd_printf(false, row, 7, "GlideRatio IIR    %2d",  opt.misc.glideRatioIIR); row++;
     }
  if (ScreenParOffset < 12) {
     lcd_printf(false, row, 7, "GPS Stable DOP    %2d",  opt.misc.gpsStableDOP); row++;
     }
  if (ScreenParOffset < 13) {
     lcd_printf(false, row, 7, "Gyro Offset Max  %3d",  opt.misc.gyroOffsetLimit1000DPS); row++;
     }
  if (ScreenParOffset < 14) {
     lcd_printf(false, row, 7, "Mag Declination  %3d",  opt.misc.magDeclinationdeg); row++;
     }
  if (ScreenParOffset < 15) {
     lcd_printf(false, row, 7, "Speaker Volume     %1d",  opt.misc.speakerVolume); row++;
     }
  if (ScreenParOffset < 16) {
     lcd_printf(false, row, 7, "Log Type        %s",  szLogType[opt.misc.logType]); row++;
     }
  if (ScreenParOffset < 16) {
     lcd_printf(false, row, 7, "Waypt radius   %5d",  opt.misc.waypointRadiusm); row++;
     }
  lcd_sendFrame();
  }


int ui_optionsEventHandler(void)  {
   if (Btn0Pressed) {
      btn_clear();
		if (ParChanged) {
  			ParChanged = 0;
         lcd_clear();
         lcd_printlnf(true, 0, "Saving options");
         opt_save();
         delayMs(1000);
  		   }
      return 1;
      }
  
   if (BtnMPressed)	{
      btn_clear();
	   ParDisplaySel = !ParDisplaySel;
      ui_displayOptions();
      return 0;
      }
	
   if (BtnLPressed) {   
      btn_clear();
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

			   case SEL_ACCEL_VAR : if (opt.kf.accelVariance >= KF_ACCEL_VARIANCE_MIN+5) opt.kf.accelVariance -= 5; 
	         break; 

			   case SEL_ZMEAS_VAR : if (opt.kf.zMeasVariance >= KF_ZMEAS_VARIANCE_MIN+10) opt.kf.zMeasVariance -= 10; 
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
	         }
		  }
		else { // ParDisplaySel == 0
         if (ParSel > 0) ParSel--;
         if (ScreenParOffset > ParSel) {
             ScreenParOffset = ParSel;
            }
		   }
      ui_displayOptions();
      return 0;
      }
   
   if (BtnRPressed) {   
      btn_clear();
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

			   case SEL_ACCEL_VAR : if (opt.kf.accelVariance <= KF_ACCEL_VARIANCE_MAX-5) opt.kf.accelVariance += 5; 
	         break; 

			   case SEL_ZMEAS_VAR : if (opt.kf.zMeasVariance <= KF_ZMEAS_VARIANCE_MAX-10) opt.kf.zMeasVariance += 10; 
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
		      }
	      }
		else { // ParDisplaySel == 0
         if (ParSel <  SEL_WPT_RADIUS) ParSel++;
         if (ScreenParOffset < (ParSel-7)) {
            ScreenParOffset = ParSel-7;
            }  
         }
      ui_displayOptions();
      return 0;
  	   }
   return 0;
   }

