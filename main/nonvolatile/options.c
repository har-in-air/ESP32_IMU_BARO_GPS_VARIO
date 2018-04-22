#include "common.h"
#include "config.h"
#include "options.h"

#define TAG "options"

OPTIONS opt;


int opt_init(void) {
   FILE *fdrd;
   char line[80];

   opt_setDefaults(); // set to defaults and override with values found in options.txt

   ESP_LOGI(TAG,"Opening spiffs options.txt file... ");
   fdrd = fopen("/spiffs/options.txt", "r");
   if (fdrd == NULL) {
      ESP_LOGI(TAG,"options.txt file not found, saving file with default values");
      opt_save();
      return 0;
      }

   KEY_VAL* pkeys = (KEY_VAL*) calloc(MAX_OPTIONS,sizeof(KEY_VAL));
   if (pkeys == NULL) {
      ESP_LOGE(TAG, "error calloc pkeys");
      return -1;
      }
   char* szToken;
   int numKeys = 0;
   while (fgets(line, 80, fdrd) != NULL) { 
      char *p = line;
      while (*p == ' ' || *p == '\t') p++;
      //  skip lines beginning with '#' or blank lines
      if (*p == '#'  ||  *p == '\r' || *p == '\n') continue;
      //ESP_LOGI(TAG,"%s", p);
      szToken = strtok(p," ,=\t[");
      if (szToken != NULL)   {
         strcpy(pkeys[numKeys].szName, szToken);
         szToken = strtok(NULL,"] ");
         if (szToken != NULL) {
            szToken = strtok(NULL," \r\n");
            if (szToken != NULL) {
               strcpy(pkeys[numKeys].szValue, szToken);
               //ESP_LOGI(TAG,"[%d]  %s =  %s",numKeys,pkeys[numKeys].szName, pkeys[numKeys].szValue);
               numKeys++;
               }
            }
         }
      }

   for (int key = 0; key < numKeys; key++) {
     // ESP_LOGI(TAG,"[%d]  %s  = %s",key, pkeys[key].szName, pkeys[key].szValue);
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
         opt.misc.logType = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.logType, LOGTYPE_NONE, LOGTYPE_IBG);
         }
      else
      if (!strcmp(pkeys[key].szName, "waypointRadiusm")) {
         opt.misc.waypointRadiusm = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.waypointRadiusm, WAYPOINT_RADIUS_M_MIN, WAYPOINT_RADIUS_M_MAX);
         }
      else
      if (!strcmp(pkeys[key].szName, "altitudeDisplay")) {
         opt.misc.altitudeDisplay = atoi(pkeys[key].szValue);
         CLAMP(opt.misc.altitudeDisplay, ALTITUDE_DISPLAY_BARO, ALTITUDE_DISPLAY_GPS);
         }
      }
   free (pkeys);

#ifdef OPT_DEBUG	
      ESP_LOGI(TAG,"VARIO CONFIGURATION");
      ESP_LOGI(TAG,"climbThresholdCps = %d", opt.vario.climbThresholdCps);
      ESP_LOGI(TAG,"zeroThresholdCps = %d", opt.vario.zeroThresholdCps);
      ESP_LOGI(TAG,"sinkThresholdCps = %d", opt.vario.sinkThresholdCps);
      ESP_LOGI(TAG,"crossoverCps = %d", opt.vario.crossoverCps);
      ESP_LOGI(TAG,"varioDisplayIIR = %d", opt.vario.varioDisplayIIR);
      ESP_LOGI(TAG,"KALMAN FILTER CONFIGURATION");
      ESP_LOGI(TAG,"accelVariance = %d", opt.kf.accelVariance);
      ESP_LOGI(TAG,"zMeasVariance = %d", opt.kf.zMeasVariance);
      ESP_LOGI(TAG,"MISCELLANEOUS CONFIGURATION");
      ESP_LOGI(TAG,"backlitSecs = %d", opt.misc.backlitSecs);
      ESP_LOGI(TAG,"gyroOffsetLimit1000DPS = %d", opt.misc.gyroOffsetLimit1000DPS);
      ESP_LOGI(TAG,"trackIntervalSecs = %d", opt.misc.trackIntervalSecs);
      ESP_LOGI(TAG,"utcOffsetMins = %d", opt.misc.utcOffsetMins);
      ESP_LOGI(TAG,"gpsStableDOP = %d", opt.misc.gpsStableDOP);
      ESP_LOGI(TAG,"glideRatioIIR = %d", opt.misc.glideRatioIIR);
      ESP_LOGI(TAG,"trackStartThresholdm = %d", opt.misc.trackStartThresholdm);
      ESP_LOGI(TAG,"magDeclinationdeg = %d", opt.misc.magDeclinationdeg);
      ESP_LOGI(TAG,"speakerVolume = %d", opt.misc.speakerVolume);
      ESP_LOGI(TAG,"logType = %d", opt.misc.logType);
      ESP_LOGI(TAG,"waypointRadiusm = %d", opt.misc.waypointRadiusm);
      ESP_LOGI(TAG,"altitudeDisplay = %d", opt.misc.altitudeDisplay);
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
   }



int opt_save() {
    FILE *fdwr;
    char buf[80];
    ssize_t nwrote;

    remove("/spiffs/options.txt");
    fdwr = fopen("/spiffs/options.txt", "wb");
    if (fdwr == NULL) return -1;

    sprintf(buf,"# Vario\r\n");
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -2;

    sprintf(buf,"climbThresholdCps [%d,%d] %d\r\n",
         VARIO_CLIMB_THRESHOLD_CPS_MIN,VARIO_CLIMB_THRESHOLD_CPS_MAX,opt.vario.climbThresholdCps);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -3;

    sprintf(buf,"zeroThresholdCps [%d,%d] %d\r\n",
         VARIO_ZERO_THRESHOLD_CPS_MIN,VARIO_ZERO_THRESHOLD_CPS_MAX,opt.vario.zeroThresholdCps);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -4;

    sprintf(buf,"sinkThresholdCps [%d,%d] %d\r\n",
         VARIO_SINK_THRESHOLD_CPS_MIN,VARIO_SINK_THRESHOLD_CPS_MAX,opt.vario.sinkThresholdCps);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -5;

    sprintf(buf,"crossoverCps [%d,%d] %d\r\n\r\n",
         VARIO_CROSSOVER_CPS_MIN,VARIO_CROSSOVER_CPS_MAX,opt.vario.crossoverCps);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -6;

    sprintf(buf,"varioDisplayIIR [%d,%d] %d\r\n\r\n",
         VARIO_DISPLAY_IIR_MIN,VARIO_DISPLAY_IIR_MAX,opt.vario.varioDisplayIIR);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -7;

    sprintf(buf,"# Kalman Filter\r\n");
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -8;

    sprintf(buf,"accelVariance [%d,%d] %d\r\n",
         KF_ACCEL_VARIANCE_MIN,KF_ACCEL_VARIANCE_MAX, opt.kf.accelVariance);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -9;

    sprintf(buf,"zMeasVariance [%d,%d] %d\r\n\r\n",
         KF_ZMEAS_VARIANCE_MIN,KF_ZMEAS_VARIANCE_MAX, opt.kf.zMeasVariance);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -10;

    sprintf(buf,"# Miscellaneous\r\n");
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -11;

    sprintf(buf,"backlitSecs [%d,%d] %d\r\n",
         BACKLIT_SECS_MIN, BACKLIT_SECS_MAX, opt.misc.backlitSecs);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -12;

    sprintf(buf,"utcOffsetMins [%d,%d] %d\r\n",
         UTC_OFFSET_MINS_MIN, UTC_OFFSET_MINS_MAX, opt.misc.utcOffsetMins);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -13;

    sprintf(buf,"gyroOffsetLimit1000DPS [%d,%d] %d\r\n",
         GYRO_OFFSET_LIMIT_1000DPS_MIN,GYRO_OFFSET_LIMIT_1000DPS_MAX, opt.misc.gyroOffsetLimit1000DPS);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -14;

    sprintf(buf,"trackIntervalSecs [%d,%d] %d\r\n",
         TRACK_INTERVAL_SECS_MIN, TRACK_INTERVAL_SECS_MAX, opt.misc.trackIntervalSecs);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -15;

    sprintf(buf,"gpsStableDOP [%d,%d] %d\r\n",
         GPS_STABLE_DOP_MIN, GPS_STABLE_DOP_MAX, opt.misc.gpsStableDOP);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -16;

    sprintf(buf,"glideRatioIIR [%d,%d] %d\r\n",
         GLIDE_RATIO_IIR_MIN, GLIDE_RATIO_IIR_MAX, opt.misc.glideRatioIIR);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -17;

    sprintf(buf,"trackStartThresholdm [%d,%d] %d\r\n",
         TRACK_START_THRESHOLD_M_MIN,TRACK_START_THRESHOLD_M_MAX, opt.misc.trackStartThresholdm);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -18;

    sprintf(buf,"magDeclinationdeg [%d,%d] %d\r\n",
          MAG_DECLINATION_DEG_MIN, MAG_DECLINATION_DEG_MAX, opt.misc.magDeclinationdeg);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -19;

    sprintf(buf,"speakerVolume [%d,%d] %d\r\n",
          SPEAKER_VOLUME_MIN, SPEAKER_VOLUME_MAX, opt.misc.speakerVolume);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -20;

    sprintf(buf,"logType [0=none,1=gps,2=ibg] %d\r\n", opt.misc.logType);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -21;

    sprintf(buf,"waypointRadiusm [%d,%d] %d\r\n",WAYPOINT_RADIUS_M_MIN, WAYPOINT_RADIUS_M_MAX, opt.misc.waypointRadiusm);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -22;

    sprintf(buf,"altitudeDisplay [%d,%d] %d\r\n", ALTITUDE_DISPLAY_GPS, ALTITUDE_DISPLAY_BARO, opt.misc.altitudeDisplay);
    nwrote =  fwrite(buf, 1, strlen(buf), fdwr);
    if (nwrote != strlen(buf)) return -22;

    fclose(fdwr);

#ifdef OPT_DEBUG
      ESP_LOGI(TAG,"------Saved options-----");
      ESP_LOGI(TAG,"VARIO CONFIGURATION");
      ESP_LOGI(TAG,"climbThresholdCps = %d", opt.vario.climbThresholdCps);
      ESP_LOGI(TAG,"zeroThresholdCps = %d", opt.vario.zeroThresholdCps);
      ESP_LOGI(TAG,"sinkThresholdCps = %d", opt.vario.sinkThresholdCps);
      ESP_LOGI(TAG,"crossoverCps = %d", opt.vario.crossoverCps);
      ESP_LOGI(TAG,"varioDisplayIIR = %d", opt.vario.varioDisplayIIR);
      ESP_LOGI(TAG,"KALMAN FILTER CONFIGURATION");
      ESP_LOGI(TAG,"accelVariance = %d", opt.kf.accelVariance);
      ESP_LOGI(TAG,"zMeasVariance = %d", opt.kf.zMeasVariance);
      ESP_LOGI(TAG,"MISCELLANEOUS CONFIGURATION");
      ESP_LOGI(TAG,"backlitSecs = %d", opt.misc.backlitSecs);
      ESP_LOGI(TAG,"gyroOffsetLimit1000DPS = %d", opt.misc.gyroOffsetLimit1000DPS);
      ESP_LOGI(TAG,"trackIntervalSecs = %d", opt.misc.trackIntervalSecs);
      ESP_LOGI(TAG,"utcOffsetMins = %d", opt.misc.utcOffsetMins);
      ESP_LOGI(TAG,"gpsStableDOP = %d", opt.misc.gpsStableDOP);
      ESP_LOGI(TAG,"glideRatioIIR = %d", opt.misc.glideRatioIIR);
      ESP_LOGI(TAG,"trackStartThresholdm = %d", opt.misc.trackStartThresholdm);
      ESP_LOGI(TAG,"magDeclinationdeg = %d", opt.misc.magDeclinationdeg);
      ESP_LOGI(TAG,"speakerVolume = %d", opt.misc.speakerVolume);
      ESP_LOGI(TAG,"logType = %d", opt.misc.logType);
      ESP_LOGI(TAG,"waypointRadiusm = %d", opt.misc.waypointRadiusm);
      ESP_LOGI(TAG,"altitudeDisplay = %d", opt.misc.altitudeDisplay);
#endif
   return 0;
   }

