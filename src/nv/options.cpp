#include "common.h"
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "options.h"
#include "ui/ui.h"

static const char* TAG = "options";

OPTIONS opt;

int opt_init(void) {
   // first set default options
   opt_setDefaults();

   // override defaults with any options found in options.txt
   ESP_LOGD(TAG,"Opening LittleFS options.txt file... ");
   File fdopt = LittleFS.open("/options.txt", FILE_READ);
   if (!fdopt) {
      ESP_LOGD(TAG,"options.txt file not found, saving file with default values");
      opt_save();
      return 0;
      }

   KEY_VAL* pkeys = (KEY_VAL*) calloc(NUM_OPTIONS,sizeof(KEY_VAL));
   if (pkeys == NULL) {
      ESP_LOGE(TAG, "error calloc pkeys");
      return -1;
      }
   String szLine;
   char* szToken;
   int numKeys = 0;
   while (fdopt.available()) {
      szLine = fdopt.readStringUntil('\n');
      ESP_LOGV(TAG, "%s", szLine.c_str());
      char *p = (char*)szLine.c_str();
      while (*p == ' ' || *p == '\t') p++;
      //  skip lines beginning with '#' or blank lines
      if (*p == '#'  ||  *p == '\r' || *p == '\n') continue;
      ESP_LOGV(TAG,"%s", p);
      szToken = strtok(p," \t[");
      if (szToken != NULL)   {
         strcpy(pkeys[numKeys].szName, szToken);
         szToken = strtok(NULL,"]\t ");
         if (szToken != NULL) {
            szToken = strtok(NULL," \r\n");
            if (szToken != NULL) {
               strcpy(pkeys[numKeys].szValue, szToken);
               ESP_LOGV(TAG,"[%d]  %s =  %s",numKeys,pkeys[numKeys].szName, pkeys[numKeys].szValue);
               numKeys++;
               }
            }
         }
      }
   fdopt.close();

   if (numKeys != NUM_OPTIONS) {
	   ESP_LOGE(TAG, "numKeys %d != NUM_OPTIONS (%d)", numKeys, NUM_OPTIONS);
      }
   for (int key = 0; key < numKeys; key++) {
      ESP_LOGV(TAG,"[%d]  %s  = %s",key, pkeys[key].szName, pkeys[key].szValue);
      if (!strcmp(pkeys[key].szName, "climbThresholdCps")) {
         opt.vario.climbThresholdCps = atoi(pkeys[key].szValue);
         CLAMP(opt.vario.climbThresholdCps,VARIO_CLIMB_THRESHOLD_CPS_MIN, VARIO_CLIMB_THRESHOLD_CPS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "zeroThresholdCps")) {
         opt.vario.zeroThresholdCps = atoi(pkeys[key].szValue);
         CLAMP(opt.vario.zeroThresholdCps,VARIO_ZERO_THRESHOLD_CPS_MIN, VARIO_ZERO_THRESHOLD_CPS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "sinkThresholdCps")) {
         opt.vario.sinkThresholdCps = atoi(pkeys[key].szValue);
         CLAMP(opt.vario.sinkThresholdCps,VARIO_SINK_THRESHOLD_CPS_MIN, VARIO_SINK_THRESHOLD_CPS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "crossoverCps")) {
         opt.vario.crossoverCps = atoi(pkeys[key].szValue);
         CLAMP(opt.vario.crossoverCps,VARIO_CROSSOVER_CPS_MIN, VARIO_CROSSOVER_CPS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "varioDisplayIIR")) {
         opt.vario.varioDisplayIIR = atoi(pkeys[key].szValue);
         CLAMP(opt.vario.varioDisplayIIR,VARIO_DISPLAY_IIR_MIN, VARIO_DISPLAY_IIR_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "accelVariance")) {
         opt.kf.accelVariance = atoi(pkeys[key].szValue);
         CLAMP(opt.kf.accelVariance,KF_ACCEL_VARIANCE_MIN, KF_ACCEL_VARIANCE_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "zMeasVariance")) {
         opt.kf.zMeasVariance = atoi(pkeys[key].szValue);
         CLAMP(opt.kf.zMeasVariance, KF_ZMEAS_VARIANCE_MIN, KF_ZMEAS_VARIANCE_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "utcOffsetMins")) {
         opt.misc.utcOffsetMins = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.utcOffsetMins, UTC_OFFSET_MINS_MIN, UTC_OFFSET_MINS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "backlitSecs")) {
         opt.misc.backlitSecs = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.backlitSecs, BACKLIT_SECS_MIN, BACKLIT_SECS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "trackStartThresholdm")) {
         opt.misc.trackStartThresholdm = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.trackStartThresholdm, TRACK_START_THRESHOLD_M_MIN, TRACK_START_THRESHOLD_M_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "trackIntervalSecs")) {
         opt.misc.trackIntervalSecs = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.trackIntervalSecs, TRACK_INTERVAL_SECS_MIN, TRACK_INTERVAL_SECS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "glideRatioIIR")) {
         opt.misc.glideRatioIIR = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.glideRatioIIR, GLIDE_RATIO_IIR_MIN, GLIDE_RATIO_IIR_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "gpsStableDOP")) {
         opt.misc.gpsStableDOP = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.gpsStableDOP, GPS_STABLE_DOP_MIN, GPS_STABLE_DOP_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "gyroOffsetLimit1000DPS")) {
         opt.misc.gyroOffsetLimit1000DPS = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.gyroOffsetLimit1000DPS, GYRO_OFFSET_LIMIT_1000DPS_MIN, GYRO_OFFSET_LIMIT_1000DPS_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "magDeclinationdeg")) {
         opt.misc.magDeclinationdeg = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.magDeclinationdeg, MAG_DECLINATION_DEG_MIN, MAG_DECLINATION_DEG_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "speakerVolume")) {
         opt.misc.speakerVolume = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.speakerVolume, SPEAKER_VOLUME_MIN, SPEAKER_VOLUME_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "logType")) {
    	  if( (!strcmp(pkeys[key].szValue, "NONE")) || (!strcmp(pkeys[key].szValue, "none"))) {
    		  opt.misc.logType = LOGTYPE_NONE;
    	  	  }
    	  else
          if( (!strcmp(pkeys[key].szValue, "GPS")) || (!strcmp(pkeys[key].szValue, "gps"))) {
        		  opt.misc.logType = LOGTYPE_GPS;
        	  }
    	  else
          if( (!strcmp(pkeys[key].szValue, "IBG")) || (!strcmp(pkeys[key].szValue, "ibg"))) {
        		  opt.misc.logType = LOGTYPE_IBG;
        	  }
         CLAMP(opt.misc.logType, LOGTYPE_NONE, LOGTYPE_IBG);
         }
      else
      if (!strcmp(pkeys[key].szName, "waypointRadiusm")) {
         opt.misc.waypointRadiusm = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.waypointRadiusm, WAYPOINT_RADIUS_M_MIN, WAYPOINT_RADIUS_M_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "altitudeDisplay")) {
    	  if( (!strcmp(pkeys[key].szValue, "BARO")) || (!strcmp(pkeys[key].szValue, "baro"))) {
    		  opt.misc.altitudeDisplay = ALTITUDE_DISPLAY_BARO;
    	  	  }
    	  else
          if( (!strcmp(pkeys[key].szValue, "GPS")) || (!strcmp(pkeys[key].szValue, "gps"))) {
        		  opt.misc.altitudeDisplay = ALTITUDE_DISPLAY_GPS;
        	  }
         CLAMP(opt.misc.altitudeDisplay, ALTITUDE_DISPLAY_BARO, ALTITUDE_DISPLAY_GPS);
         }
      else
      if (!strcmp(pkeys[key].szName, "btMsgType")) {
    	  if( (!strcmp(pkeys[key].szValue, "LK8")) || (!strcmp(pkeys[key].szValue, "lk8"))) {
    		  opt.misc.btMsgType = BT_MSG_LK8EX1;
    	  	  }
    	  else
          if( (!strcmp(pkeys[key].szValue, "XCT")) || (!strcmp(pkeys[key].szValue, "xct"))) {
        		  opt.misc.btMsgType = BT_MSG_XCTRC;
        	  }
         CLAMP(opt.misc.btMsgType, BT_MSG_LK8EX1, BT_MSG_XCTRC);
         }
      else
      if (!strcmp(pkeys[key].szName, "btMsgFreqHz")) {
          opt.misc.btMsgFreqHz = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.btMsgFreqHz, BT_MSG_FREQ_HZ_MIN, BT_MSG_FREQ_HZ_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "lcdContrast")) {
          opt.misc.lcdContrast = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.lcdContrast, LCD_CONTRAST_MIN, LCD_CONTRAST_MAX);
         }
      }
   free (pkeys);

#ifdef OPT_DEBUG	
      ESP_LOGD(TAG,"VARIO CONFIGURATION");
      ESP_LOGD(TAG,"climbThresholdCps = %d", opt.vario.climbThresholdCps);
      ESP_LOGD(TAG,"zeroThresholdCps = %d", opt.vario.zeroThresholdCps);
      ESP_LOGD(TAG,"sinkThresholdCps = %d", opt.vario.sinkThresholdCps);
      ESP_LOGD(TAG,"crossoverCps = %d", opt.vario.crossoverCps);
      ESP_LOGD(TAG,"varioDisplayIIR = %d", opt.vario.varioDisplayIIR);
      ESP_LOGD(TAG,"KALMAN FILTER CONFIGURATION");
      ESP_LOGD(TAG,"accelVariance = %d", opt.kf.accelVariance);
      ESP_LOGD(TAG,"zMeasVariance = %d", opt.kf.zMeasVariance);
      ESP_LOGD(TAG,"MISCELLANEOUS CONFIGURATION");
      ESP_LOGD(TAG,"backlitSecs = %d", opt.misc.backlitSecs);
      ESP_LOGD(TAG,"gyroOffsetLimit1000DPS = %d", opt.misc.gyroOffsetLimit1000DPS);
      ESP_LOGD(TAG,"trackIntervalSecs = %d", opt.misc.trackIntervalSecs);
      ESP_LOGD(TAG,"utcOffsetMins = %d", opt.misc.utcOffsetMins);
      ESP_LOGD(TAG,"gpsStableDOP = %d", opt.misc.gpsStableDOP);
      ESP_LOGD(TAG,"glideRatioIIR = %d", opt.misc.glideRatioIIR);
      ESP_LOGD(TAG,"trackStartThresholdm = %d", opt.misc.trackStartThresholdm);
      ESP_LOGD(TAG,"magDeclinationdeg = %d", opt.misc.magDeclinationdeg);
      ESP_LOGD(TAG,"speakerVolume = %d", opt.misc.speakerVolume);
      ESP_LOGD(TAG,"logType = %s", szLogType[opt.misc.logType]);
      ESP_LOGD(TAG,"waypointRadiusm = %d", opt.misc.waypointRadiusm);
      ESP_LOGD(TAG,"btMsgType = %s", szBtMsgType[opt.misc.btMsgType]);
      ESP_LOGD(TAG,"btMsgFreqHz = %d", opt.misc.btMsgFreqHz);
      ESP_LOGD(TAG,"lcdContrast = %d", opt.misc.lcdContrast);
#endif
   return 0;
	}


// If the options.txt file is not found, a new options.txt file will be saved with 
// default values.
// So to set 'factory defaults', access the gpsvario webpage configuration and delete 
// the options.txt file.

void opt_setDefaults() {
	opt.vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
	opt.vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
	opt.vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT;
	opt.vario.crossoverCps = VARIO_CROSSOVER_CPS_DEFAULT;
	opt.vario.varioDisplayIIR = VARIO_DISPLAY_IIR_DEFAULT;
	
	opt.kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
	opt.kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

	opt.misc.utcOffsetMins = UTC_OFFSET_MINS_DEFAULT;
	opt.misc.backlitSecs = BACKLIT_SECS_DEFAULT;
	opt.misc.trackIntervalSecs = TRACK_INTERVAL_SECS_DEFAULT;
	opt.misc.glideRatioIIR = GLIDE_RATIO_IIR_DEFAULT;
	opt.misc.gpsStableDOP = GPS_STABLE_DOP_DEFAULT;
	opt.misc.gyroOffsetLimit1000DPS = GYRO_OFFSET_LIMIT_1000DPS_DEFAULT;
	opt.misc.trackStartThresholdm = TRACK_START_THRESHOLD_M_DEFAULT;
	opt.misc.magDeclinationdeg = MAG_DECLINATION_DEG_DEFAULT;
	opt.misc.speakerVolume = SPEAKER_VOLUME_DEFAULT;
	opt.misc.logType = LOGTYPE_NONE;
	opt.misc.waypointRadiusm = WAYPOINT_RADIUS_M_DEFAULT;
	opt.misc.altitudeDisplay = ALTITUDE_DISPLAY_GPS;
	opt.misc.btMsgType = BT_MSG_LK8EX1;
	opt.misc.btMsgFreqHz = BT_MSG_FREQ_HZ_DEFAULT;
	opt.misc.lcdContrast = LCD_CONTRAST_DEFAULT;
   }



int opt_save() {
    File f;
    char buf[80];
    ssize_t nwrote;

    LittleFS.remove("/options.txt");
    File fdopt = LittleFS.open("/options.txt", FILE_WRITE);
    if (!fdopt) {
      ESP_LOGE(TAG, "Error opening options.txt to write");
      return -1;
      }

    sprintf(buf,"# Vario\r\n");
    nwrote =  fdopt.print(buf);
    if (nwrote != strlen(buf)) return -2;

    sprintf(buf,"climbThresholdCps [%d,%d] %d\r\n",
         VARIO_CLIMB_THRESHOLD_CPS_MIN,VARIO_CLIMB_THRESHOLD_CPS_MAX,opt.vario.climbThresholdCps);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -3;

    sprintf(buf,"zeroThresholdCps [%d,%d] %d\r\n",
         VARIO_ZERO_THRESHOLD_CPS_MIN,VARIO_ZERO_THRESHOLD_CPS_MAX,opt.vario.zeroThresholdCps);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -4;

    sprintf(buf,"sinkThresholdCps [%d,%d] %d\r\n",
         VARIO_SINK_THRESHOLD_CPS_MIN,VARIO_SINK_THRESHOLD_CPS_MAX,opt.vario.sinkThresholdCps);
    nwrote =  fdopt.print(buf);
    if (nwrote != strlen(buf)) return -5;

    sprintf(buf,"crossoverCps [%d,%d] %d\r\n",
         VARIO_CROSSOVER_CPS_MIN,VARIO_CROSSOVER_CPS_MAX,opt.vario.crossoverCps);
    nwrote = fdopt.print(buf);;
    if (nwrote != strlen(buf)) return -6;

    sprintf(buf,"varioDisplayIIR [%d,%d] %d\r\n\r\n",
         VARIO_DISPLAY_IIR_MIN,VARIO_DISPLAY_IIR_MAX,opt.vario.varioDisplayIIR);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -7;

    sprintf(buf,"# Kalman Filter\r\n");
    nwrote =  fdopt.print(buf);
    if (nwrote != strlen(buf)) return -8;

    sprintf(buf,"accelVariance [%d,%d] %d\r\n",
         KF_ACCEL_VARIANCE_MIN,KF_ACCEL_VARIANCE_MAX, opt.kf.accelVariance);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -9;

    sprintf(buf,"zMeasVariance [%d,%d] %d\r\n\r\n",
         KF_ZMEAS_VARIANCE_MIN,KF_ZMEAS_VARIANCE_MAX, opt.kf.zMeasVariance);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -10;

    sprintf(buf,"# Miscellaneous\r\n");
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -11;

    sprintf(buf,"backlitSecs [%d,%d] %d\r\n",
         BACKLIT_SECS_MIN, BACKLIT_SECS_MAX, opt.misc.backlitSecs);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -12;

    sprintf(buf,"utcOffsetMins [%d,%d] %d\r\n",
         UTC_OFFSET_MINS_MIN, UTC_OFFSET_MINS_MAX, opt.misc.utcOffsetMins);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -13;

    sprintf(buf,"gyroOffsetLimit1000DPS [%d,%d] %d\r\n",
         GYRO_OFFSET_LIMIT_1000DPS_MIN,GYRO_OFFSET_LIMIT_1000DPS_MAX, opt.misc.gyroOffsetLimit1000DPS);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -14;

    sprintf(buf,"trackIntervalSecs [%d,%d] %d\r\n",
         TRACK_INTERVAL_SECS_MIN, TRACK_INTERVAL_SECS_MAX, opt.misc.trackIntervalSecs);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -15;

    sprintf(buf,"gpsStableDOP [%d,%d] %d\r\n",
         GPS_STABLE_DOP_MIN, GPS_STABLE_DOP_MAX, opt.misc.gpsStableDOP);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -16;

    sprintf(buf,"glideRatioIIR [%d,%d] %d\r\n",
         GLIDE_RATIO_IIR_MIN, GLIDE_RATIO_IIR_MAX, opt.misc.glideRatioIIR);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -17;

    sprintf(buf,"trackStartThresholdm [%d,%d] %d\r\n",
         TRACK_START_THRESHOLD_M_MIN,TRACK_START_THRESHOLD_M_MAX, opt.misc.trackStartThresholdm);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -18;

    sprintf(buf,"magDeclinationdeg [%d,%d] %d\r\n",
          MAG_DECLINATION_DEG_MIN, MAG_DECLINATION_DEG_MAX, opt.misc.magDeclinationdeg);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -19;

    sprintf(buf,"speakerVolume [%d,%d] %d\r\n",
          SPEAKER_VOLUME_MIN, SPEAKER_VOLUME_MAX, opt.misc.speakerVolume);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -20;

    sprintf(buf,"logType [NONE,GPS,IBG] %s\r\n", szLogType[opt.misc.logType]);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -21;

    sprintf(buf,"waypointRadiusm [%d,%d] %d\r\n",WAYPOINT_RADIUS_M_MIN, WAYPOINT_RADIUS_M_MAX, opt.misc.waypointRadiusm);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -22;

    sprintf(buf,"altitudeDisplay [GPS,BARO] %s\r\n", szAltDisplayType[opt.misc.altitudeDisplay]);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -23;

    sprintf(buf,"btMsgType [LK8,XCT] %s\r\n", szBtMsgType[opt.misc.btMsgType]);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -24;

    sprintf(buf,"btMsgFreqHz [%d,%d] %d\r\n", BT_MSG_FREQ_HZ_MIN, BT_MSG_FREQ_HZ_MAX, opt.misc.btMsgFreqHz);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -25;

    sprintf(buf,"lcdContrast [%d,%d] %d\r\n", LCD_CONTRAST_MIN, LCD_CONTRAST_MAX, opt.misc.lcdContrast);
    nwrote = fdopt.print(buf);
    if (nwrote != strlen(buf)) return -26;
    fdopt.close();

#ifdef OPT_DEBUG
      ESP_LOGD(TAG,"------Saved options-----");
      ESP_LOGD(TAG,"VARIO CONFIGURATION");
      ESP_LOGD(TAG,"climbThresholdCps = %d", opt.vario.climbThresholdCps);
      ESP_LOGD(TAG,"zeroThresholdCps = %d", opt.vario.zeroThresholdCps);
      ESP_LOGD(TAG,"sinkThresholdCps = %d", opt.vario.sinkThresholdCps);
      ESP_LOGD(TAG,"crossoverCps = %d", opt.vario.crossoverCps);
      ESP_LOGD(TAG,"varioDisplayIIR = %d", opt.vario.varioDisplayIIR);
      ESP_LOGD(TAG,"KALMAN FILTER CONFIGURATION");
      ESP_LOGD(TAG,"accelVariance = %d", opt.kf.accelVariance);
      ESP_LOGD(TAG,"zMeasVariance = %d", opt.kf.zMeasVariance);
      ESP_LOGD(TAG,"MISCELLANEOUS CONFIGURATION");
      ESP_LOGD(TAG,"backlitSecs = %d", opt.misc.backlitSecs);
      ESP_LOGD(TAG,"gyroOffsetLimit1000DPS = %d", opt.misc.gyroOffsetLimit1000DPS);
      ESP_LOGD(TAG,"trackIntervalSecs = %d", opt.misc.trackIntervalSecs);
      ESP_LOGD(TAG,"utcOffsetMins = %d", opt.misc.utcOffsetMins);
      ESP_LOGD(TAG,"gpsStableDOP = %d", opt.misc.gpsStableDOP);
      ESP_LOGD(TAG,"glideRatioIIR = %d", opt.misc.glideRatioIIR);
      ESP_LOGD(TAG,"trackStartThresholdm = %d", opt.misc.trackStartThresholdm);
      ESP_LOGD(TAG,"magDeclinationdeg = %d", opt.misc.magDeclinationdeg);
      ESP_LOGD(TAG,"speakerVolume = %d", opt.misc.speakerVolume);
      ESP_LOGD(TAG,"logType = %s", szLogType[opt.misc.logType]);
      ESP_LOGD(TAG,"waypointRadiusm = %d", opt.misc.waypointRadiusm);
      ESP_LOGD(TAG,"altitudeDisplay = %s", szAltDisplayType[opt.misc.altitudeDisplay]);
      ESP_LOGD(TAG,"btMsgType = %s", szBtMsgType[opt.misc.btMsgType]);
      ESP_LOGD(TAG,"btMsgFreqHz = %d", opt.misc.btMsgFreqHz);
      ESP_LOGD(TAG,"lcdContrast = %d", opt.misc.lcdContrast);
#endif
   return 0;
   }

