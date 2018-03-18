#include "common.h"
#include "config.h"
#include "lcd7565.h"
#include "ui.h"
#include "myadc.h"
#include "gps.h"
#include "ms5611.h"
#include "options.h"
#include "cct.h"

#define TAG "ui"

bool IsSpeakerEnabled  = true;
bool IsGpsFixStable = true;
bool IsTrackActive = true;
bool IsLcdBkltEnabled = false;
bool IsLogging = false;
bool IsFlashDisplayRequired = false;
bool IsGpsHeading = true;

int32_t StartLatDeg7, StartLonDeg7;
uint32_t StartTowmS;

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


void ui_printAltitude(int page, int col, int32_t nAlt) {
   int cnt;
	char szBuf[5];
   CLAMP(nAlt,MIN_ALTITUDE_M,MAX_ALTITUDE_M);
	szBuf[4] = '\0';
	szBuf[2] = ' ';
	szBuf[1] = ' ';
	szBuf[0] = ' ';
   if (nAlt == 0){
		szBuf[3] = '0';
		}
   else
   if (nAlt < 0){
      nAlt = -nAlt;
      cnt = 3;
      while (nAlt) {
	   	szBuf[cnt--] = (nAlt%10) + 48;
         nAlt = nAlt/10;
			}
      szBuf[cnt--] = '-';
      }
   else {
      cnt = 3;
      while (nAlt) {
			szBuf[cnt--] = (nAlt%10) + 48;
         nAlt = nAlt/10;
			}
       }
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printDistance(int page, int col, int distanceM) {
   int cnt;
	char szBuf[4];
   CLAMP(distanceM,MIN_DISTANCE_M,MAX_DISTANCE_M);
   distanceM = (distanceM + 5)/10;
	lcd_setFramePos(page,col+40);
	lcd_putChar(48+ (distanceM%10));
	distanceM = distanceM/10;
	lcd_setFramePos(page,col+33);
	lcd_putChar(48+(distanceM%10));
	distanceM = distanceM/10L;
	szBuf[3] = '\0';
	szBuf[1] = ' ';
	szBuf[0] = ' ';
   if (distanceM == 0){
		szBuf[2] = '0';
		}
   else {
      cnt = 2;
      while (distanceM) {
			szBuf[cnt--] = 48 + (distanceM%10);
         distanceM = distanceM/10;
			}
      }
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
   int  cnt;
	char  szBuf[4];
	szBuf[3] = '\0';
	szBuf[1] = ' ';
	szBuf[0] = ' ';
	if (nVel > MAX_VELOCITY_KPH) {
		szBuf[2] = '-';
		}
	else {
	   if (nVel == 0){
			szBuf[2] = '0';
			}
	   else {
	      cnt = 2;
	      while (nVel) {
				szBuf[cnt--] = (nVel%10) + 48;
	         nVel = nVel/10;
				}
	      }
		}
	lcd_printSzLNum(page,col,szBuf);
   }


void ui_printGlideRatio(int page, int col, int nGr) {
	char  szBuf[3];
	szBuf[2] = '\0';
	if (nGr > 999) {
	   lcd_setFramePos(page,col+24);
	   lcd_putChar('+');
		szBuf[1] = '+';
		szBuf[0] = '+';
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

uint8_t gBuf[128];

void ui_printCompassHeadingAnalog(int isGps, int page, int col, int velkph, int heading) {
	int nrow,ncol,inx;
	int tblOffset = 0;
	lcd_setFramePos(page - 1, col+13);

   if (isGps) {
      FrameBuf[128*FramePage + FrameCol+1] = 0x3C;
      FrameBuf[128*FramePage + FrameCol+2] = 0x7C;
      FrameBuf[128*FramePage + FrameCol+3] = 0x3C;  
      // if very low velocity, gps heading is uncertain, blank out display
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
	for (inx = 0; inx < 128; inx++) gBuf[inx] = gCompassTbl[tblOffset++];

	inx = 0;
	for (nrow = 0; nrow < 4; nrow++)  {
		lcd_setFramePos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++){
         FrameBuf[128*FramePage + FrameCol+ ncol] = gBuf[inx++];
         }
		}
	}


#include "bearing.txt"

void ui_printBearingAnalog(int page, int col, int bearing) {
	int  nrow,ncol,inx;
   int tblOffset;
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

	for (inx = 0; inx < 16; inx++) gBuf[40+inx] |= gBearingTbl[tblOffset++];
	for (inx = 0; inx < 16; inx++) gBuf[72+inx] |= gBearingTbl[tblOffset++];

	inx = 0;
	for (nrow = 0; nrow < 4; nrow++)  {
		lcd_setFramePos(page+nrow, col);
	 	for (ncol = 0; ncol < 32; ncol++) {
         FrameBuf[128*FramePage + FrameCol + ncol] = gBuf[inx++];
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
      lcd_putChar('P');
	   lcd_putChar('M');
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
         lcd_putChar('A');
	      lcd_putChar('M');
		   szBuf[1] = nHrs+48;
		   szBuf[0] = ' ';
		   }
   	else {
         lcd_putChar('A');
	      lcd_putChar('M');
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


void ui_trackTime(int32_t startmS, int32_t currentmS, int32_t* pHrs, int32_t* pMins) {
   int32_t elapsedSecs = ((currentmS - startmS)+500)/1000;
   *pHrs = elapsedSecs/60;
   *pMins = elapsedSecs%60;
   }



void ui_updateFlightDisplay(NAV_PVT* pn, TRACK* ptrk) {
  // uint32_t marker = cct_setMarker();
   lcd_clearFrame();
   if (opt.misc.logType == LOGTYPE_IBG) {
      lcd_printSz(0,60, IsLogging ? "IBG" : "ibg");
      }
   else
   if (opt.misc.logType == LOGTYPE_GPS) {
      lcd_printSz(0,60, IsTrackActive ? "GPS" : "gps");
      }
   int bv = (int)(adc_batteryVoltage()+50)/100;
   ui_printPosDOP(6,100, (pn->nav.posDOP+50)/100);
   ui_printBatteryStatus(7, 110, bv);
   ui_printSpkrStatus(7,100, IsSpeakerEnabled);
   ui_printAltitude(0,0,(pn->nav.heightMSLmm + 500)/1000);
   lcd_printSz(1,45,"M");
   ui_printDistance(0,82,0);
   lcd_printSz(1,116,"KM");
   ui_printClimbRate(2,0,INTEGER_ROUNDUP(iirClimbrateCps));
   lcd_printSz(3,34,"MS");
   int gpsClimbrate = pn->nav.velDownmmps < 0 ? (pn->nav.velDownmmps - 50)/100 : (pn->nav.velDownmmps + 50)/100;
   lcd_printf(false,2,55, "%c%d.%d", gpsClimbrate >= 0 ? '+' : '-', ABS(gpsClimbrate)/10, ABS(gpsClimbrate)%10);
   int horzVelKph = (pn->nav.groundSpeedmmps*36)/10000;
   ui_printVelocity(4,0,horzVelKph);
   lcd_printSz(5,34,"KH");
//   ui_printGlideRatio(6,0, iirClimbrateCps < 0 ? (int)(grNew*10.0f) : 9999);
   ui_printGlideRatio(6,0, iirClimbrateCps < 0 ? (int)(glideRatio*10.0f) : 9999);
   lcd_printSz(7,23,"GR");
   int year,month,day,hour,minute;
   gps_localDateTime(pn,&year,&month,&day,&hour,&minute);
   ui_printRealTime(2,93,hour,minute);
   int elapsedHrs = 0;
   int elapsedMins = 0;
   if (IsTrackActive) {
      ui_trackTime(ptrk->startTowmS, pn->nav.timeOfWeekmS, &elapsedHrs, &elapsedMins);
      }
   ui_printTrackTime(4,93,elapsedHrs,elapsedMins);
   if (IsGpsHeading) {
      int32_t headingDeg = pn->nav.headingDeg5/100000; // gps course heading
      ui_printCompassHeadingAnalog(true,4,55,horzVelKph, headingDeg);
      }
   else{
      int32_t headingDeg = INTEGER_ROUNDUP(yawDeg);// magnetic compass heading
      headingDeg  -= (int32_t)opt.misc.magDeclinationdeg; 
      headingDeg = (headingDeg + 360)%360;
      ui_printCompassHeadingAnalog(false,4,55,0, headingDeg);
      } 
   int bearingDeg = gps_bearingDeg(pn->nav.latDeg7,pn->nav.lonDeg7, ptrk->startLatDeg7, ptrk->startLonDeg7);
   ui_printBearingAnalog(4,55, bearingDeg);
   
   if (IsFlashDisplayRequired) {
      IsFlashDisplayRequired = false;
      lcd_invertFrame();
      }
   lcd_sendFrame();
//   uint32_t eus = cct_elapsedUs(marker);
  // ESP_LOGI(TAG,"updateFlightDisplay = %dus", eus);
   }

