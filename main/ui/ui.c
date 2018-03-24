#include "common.h"
#include "config.h"
#include "lcd7565.h"
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

void ui_printHeadingAnalog(int isGps, int page, int col, int velkph, int heading) {
	int nrow,ncol,inx;
	int tblOffset = 0;
	lcd_setFramePos(page - 1, col+13);

   if (isGps) {
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
		if (heading <= 11) tblOffset = 0;
		else
		if (heading <= 34) tblOffset = 1;
		else
		if (heading <= 56) tblOffset = 2;
		else
		if (heading <= 79) tblOffset = 3;
		else
		if (heading <= 101) tblOffset = 4;
		else
		if (heading <= 124) tblOffset = 5;
		else
		if (heading <= 146) tblOffset = 6;
		else
		if (heading <= 169) tblOffset = 7;
		else
		if (heading <= 191) tblOffset = 8;
		else
		if (heading <= 214) tblOffset = 9;
		else
		if (heading <= 236) tblOffset = 10;
		else
		if (heading <= 259) tblOffset = 11;
		else
		if (heading <= 281) tblOffset = 12;
		else
		if (heading <= 304) tblOffset = 13;
		else
		if (heading <= 326) tblOffset = 14;
		else
		if (heading <= 349) tblOffset = 15;
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

void ui_printBearingAnalog(int page, int col,int velkph, int bearing) {
	int  nrow,ncol,inx;
   int tblOffset;
   // if very low velocity, gps heading is uncertain, do not display bearing
	if (velkph < 2) { 
		return;
	   }
	bearing = bearing % 360;
	if (bearing <= 11) tblOffset = 0;
	else
	if (bearing <= 34) tblOffset = 1;
	else
	if (bearing <= 56) tblOffset = 2;
	else
	if (bearing <= 79) tblOffset = 3;
	else
	if (bearing <= 101) tblOffset = 4;
	else
	if (bearing <= 124) tblOffset = 5;
	else
	if (bearing <= 146) tblOffset = 6;
	else
	if (bearing <= 169) tblOffset = 7;
	else
	if (bearing <= 191) tblOffset = 8;
	else
	if (bearing <= 214) tblOffset = 9;
	else
	if (bearing <= 236) tblOffset = 10;
	else
	if (bearing <= 259) tblOffset = 11;
	else
	if (bearing <= 281) tblOffset = 12;
	else
	if (bearing <= 304) tblOffset = 13;
	else
	if (bearing <= 326) tblOffset = 14;
	else
	if (bearing <= 349) tblOffset = 15;
	else tblOffset = 0;
	tblOffset <<= 5;

	//for (inx = 0; inx < 16; inx++) compassBuf[40+inx] |= gBearingTbl[tblOffset++];
	//for (inx = 0; inx < 16; inx++) compassBuf[72+inx] |= gBearingTbl[tblOffset++];
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
      lcd_printf(false, page, col,"%02d",start);
	   }
    lcd_printf(false,page, col+24,"%02d",end);
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
   gps_localDateTime(pn,&year,&month,&day,&hour,&minute);
   ui_printRealTime(2,93,hour,minute);

   if (!IsGpsHeading) { // magnetic compass heading
      int32_t compassDeg = INTEGER_ROUNDUP(YawDeg);
      compassDeg  -= (int32_t)opt.misc.magDeclinationdeg; 
      compassDeg = (compassDeg + 360)%360;
      ui_printHeadingAnalog(false,4,55,0, compassDeg);
      }

   if (IsGpsFixStable) {
      pTrk->distanceFromStartm = gps_haversineDistancem(lat, lon, pTrk->startLatdeg, pTrk->startLondeg);
      if ((!IsTrackActive) && (pTrk->distanceFromStartm >= opt.misc.trackStartThresholdm)) {
         IsTrackActive = true;
         pTrk->startTowmS = pn->nav.timeOfWeekmS;
         pTrk->year = pn->nav.utcYear;
         pTrk->month = pn->nav.utcMonth;
         pTrk->day = pn->nav.utcDay;
         pTrk->hour = pn->nav.utcHour;
         pTrk->minute = pn->nav.utcMinute;
         }
      int32_t vn = pn->nav.velNorthmmps;
      int32_t ve = pn->nav.velEastmmps;
      int32_t gpsCourseHeadingDeg = 90 - (int32_t)(atan2((float)vn, (float)ve)*_180_DIV_PI); // course over ground (motion heading)
      gpsCourseHeadingDeg = (gpsCourseHeadingDeg + 360)%360;
      float horzVelmmps = sqrt((float)(vn*vn + ve*ve));
      int32_t horzVelKph = (int32_t)(horzVelmmps*0.0036f + 0.5f);
      ui_printVelocity(4,0,horzVelKph);
      lcd_printSz(5,34,"kh");
      static float glideRatio = 1.0f;
      if (pn->nav.velDownmmps > 0) {
         float glideRatioNew = horzVelmmps/(float)pn->nav.velDownmmps;
         glideRatio = (glideRatio*(float)opt.misc.glideRatioIIR + glideRatioNew*(float)(100-opt.misc.glideRatioIIR))/100.0f;
         ui_printGlideRatio(6,0,(int)(glideRatio*10.0f + 0.5f));
         }
      else {
         ui_printGlideRatio(6,0,1000);
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
         ui_printHeadingAnalog(true,4,55,horzVelKph, gpsCourseHeadingDeg);
         }
      if (IsRouteActive) { // show relative bearing and distance to next waypoint
         if (pRoute->nextWptInx >= pRoute->numWpts) {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg) - gpsCourseHeadingDeg;
            bearingDeg = (bearingDeg + 360)%360;
            distancem = gps_haversineDistancem(lat, lon, pRoute->wpt[pRoute->numWpts-1].latdeg, pRoute->wpt[pRoute->numWpts-1].londeg);
            ui_printRouteSegment(3, 52, pRoute->numWpts-1, pRoute->numWpts-1);
            }
         else {
            bearingDeg = gps_bearingDeg(lat, lon, pRoute->wpt[pRoute->nextWptInx].latdeg, pRoute->wpt[pRoute->nextWptInx].londeg) - gpsCourseHeadingDeg;
            bearingDeg = (bearingDeg + 360)%360;
            distancem = gps_haversineDistancem(lat, lon, pRoute->wpt[pRoute->nextWptInx].latdeg, pRoute->wpt[pRoute->nextWptInx].londeg);
            ui_printRouteSegment(3, 52, pRoute->nextWptInx-1, pRoute->nextWptInx);
            if (distancem < pRoute->wpt[pRoute->nextWptInx].radiusm) {
               ui_alarmWaypointReached();
               pRoute->nextWptInx++;
               }
            }
         }
      else { // no route, show relative bearing and distance to start
         bearingDeg = gps_bearingDeg(lat, lon, pTrk->startLatdeg, pTrk->startLondeg) - gpsCourseHeadingDeg;
         bearingDeg = (bearingDeg + 360)%360;
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


int ui_saveLog(TRACK* pTrk) {
   FILE *fdwr;
   char buf[80];
   ssize_t nwrote;

   sprintf(buf,"/spiffs/%04d%02d%02d_%02d%02d.txt", pTrk->year, pTrk->month, pTrk->day, pTrk->hour, pTrk->minute);
   fdwr = fopen(buf, "wb");
   if (fdwr == NULL) return -1;

   sprintf(buf,"UTC Start %04d/%04d/%04d %02d:%02d\r\n", pTrk->year, pTrk->month, pTrk->day, pTrk->hour, pTrk->minute);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Duration %02d:%02d\r\n", pTrk->elapsedHours, pTrk->elapsedMinutes);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Start Latitude %f Longitude %f\r\n", pTrk->startLatdeg, pTrk->startLondeg);
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Start altitude  %dm\r\n",(int)(pTrk->startAltm+0.5f));
   nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
   if (nwrote != strlen(buf)) return -2;

   sprintf(buf,"Max altitude  %dm\r\n", (int)(pTrk->maxAltm+0.5f));
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

