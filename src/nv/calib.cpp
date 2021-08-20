#include <Arduino.h>
#include <SPIFFS.h>
#include "common.h"
#include "config.h"
#include "calib.h"

static const char* TAG = "calib";

CALIB calib;

bool IsCalibrated = false;

int calib_init(void) {
   calib_setDefaults(); // set to defaults, override with calib.txt values

   ESP_LOGI(TAG,"Opening spiffs calib.txt file... ");
   File fdcal = SPIFFS.open("/calib.txt", FILE_READ);
   if (!fdcal) {
      ESP_LOGI(TAG,"calib.txt file not found, creating with defaults");
      calib_save();
      IsCalibrated = false;
      return 0;
      }

   KEY_VAL* pkeys = (KEY_VAL*) calloc(MAX_CALIB_PARAMS,sizeof(KEY_VAL));
   if (pkeys == NULL) {
      ESP_LOGE(TAG, "error calloc pkeys");
      fdcal.close();
      IsCalibrated = false;
      return -1;
      }
   String szLine;
   char* szToken;
   int numKeys = 0;
   while (fdcal.available()) {
      szLine = fdcal.readStringUntil('\n');
      ESP_LOGD(TAG, "%s", szLine.c_str());
      char *p = (char*)szLine.c_str();
      while (*p == ' ' || *p == '\t') p++;
      //  skip lines beginning with '#' or blank lines
      if (*p == '#'  ||  *p == '\r' || *p == '\n') continue;
      ESP_LOGI(TAG,"%s", p);
      szToken = strtok(p," ,=\t");
      if (szToken != NULL)   {
         strcpy(pkeys[numKeys].szName, szToken);
         szToken = strtok(NULL," \t[\r\n");
         if (szToken != NULL) {
            strcpy(pkeys[numKeys].szValue, szToken);
            //ESP_LOGI(TAG,"[%d]  %s =  %s",numKeys,pkeys[numKeys].szName, pkeys[numKeys].szValue);
            numKeys++;
            }
         }
      }
   fdcal.close();

   for (int key = 0; key < numKeys; key++) {
       ESP_LOGI(TAG,"[%d]  %s  = %s",key, pkeys[key].szName, pkeys[key].szValue);
      if (!strcmp(pkeys[key].szName, "axBias")) {
         calib.axBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "ayBias")) {
         calib.ayBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "azBias")) {
         calib.azBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "gxBias")) {
         calib.gxBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "gyBias")) {
         calib.gyBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "gzBias")) {
         calib.gzBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "mxBias")) {
         calib.mxBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "myBias")) {
         calib.myBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "mzBias")) {
         calib.mzBias = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "mxSens")) {
         calib.mxSens = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "mySens")) {
         calib.mySens = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      else
      if (!strcmp(pkeys[key].szName, "mzSens")) {
         calib.mzSens = (int16_t) strtol(pkeys[key].szValue, (char **)NULL, 10);
         }
      }
   free (pkeys);
   IsCalibrated = ( (calib.axBias == 0 && calib.ayBias == 0 && calib.azBias == 0) ||
                    (calib.mxBias == 0 && calib.myBias == 0 && calib.mzBias == 0) ) ? false : true;

#ifdef CALIB_DEBUG	
	ESP_LOGI(TAG, "IsCalibrated = %d", IsCalibrated);
	ESP_LOGI(TAG,"ACCEL,GYRO,MAG calibration parameters read from file");
	ESP_LOGI(TAG,"axBias = %d", calib.axBias);
	ESP_LOGI(TAG,"ayBias = %d", calib.ayBias);
	ESP_LOGI(TAG,"azBias = %d", calib.azBias);
	ESP_LOGI(TAG,"gxBias = %d", calib.gxBias);
	ESP_LOGI(TAG,"gyBias = %d", calib.gyBias);
	ESP_LOGI(TAG,"gzBias = %d", calib.gzBias);
	ESP_LOGI(TAG,"mxBias = %d", calib.mxBias);
	ESP_LOGI(TAG,"myBias = %d", calib.myBias);
	ESP_LOGI(TAG,"mzBias = %d", calib.mzBias);
	ESP_LOGI(TAG,"mxSens = %d", calib.mxSens);
	ESP_LOGI(TAG,"mySens = %d", calib.mySens);
	ESP_LOGI(TAG,"mzSens = %d", calib.mzSens);
#endif
   return 0;
	}


void calib_setDefaults() {
	calib.axBias = 0;
	calib.ayBias = 0;
	calib.azBias = 0;
	calib.gxBias = 0;
	calib.gyBias = 0;
	calib.gzBias = 0;
	calib.mxBias = 0;
	calib.myBias = 0;
	calib.mzBias = 0;
	calib.mxSens = 280;
	calib.mySens = 280;
	calib.mzSens = 280;
   }


int calib_save() {
   char buf[80];
   ssize_t nwrote;

   ESP_LOGD(TAG, "deleting calib.txt");
   SPIFFS.remove("/calib.txt");
   ESP_LOGD(TAG, "Opening calib.txt to write");
   File fdcal = SPIFFS.open("/calib.txt", FILE_WRITE);
   if (!fdcal) {
      ESP_LOGE(TAG, "Error opening calib.txt to write");
      return -1;
      }

    sprintf(buf,"axBias %d\r\n", calib.axBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -2;

    sprintf(buf,"ayBias %d\r\n", calib.ayBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -3;

    sprintf(buf,"azBias %d\r\n", calib.azBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -4;

    sprintf(buf,"gxBias %d\r\n", calib.gxBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -5;

    sprintf(buf,"gyBias %d\r\n", calib.gyBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -6;

    sprintf(buf,"gzBias %d\r\n", calib.gzBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -7;

    sprintf(buf,"mxBias %d\r\n", calib.mxBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -8;

    sprintf(buf,"myBias %d\r\n", calib.myBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -9;

    sprintf(buf,"mzBias %d\r\n", calib.mzBias);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -10;

    sprintf(buf,"mxSens %d\r\n", calib.mxSens);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -11;

    sprintf(buf,"mySens %d\r\n", calib.mySens);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -2;

    sprintf(buf,"mzSens %d\r\n", calib.mzSens);
    nwrote = fdcal.print(buf);
    if (nwrote != strlen(buf)) return -2;

    fdcal.close();

#ifdef CALIB_DEBUG
   ESP_LOGI(TAG,"### calibration params saved to /calib.txt");
	ESP_LOGI(TAG,"axBias = %d", calib.axBias);
	ESP_LOGI(TAG,"ayBias = %d", calib.ayBias);
	ESP_LOGI(TAG,"azBias = %d", calib.azBias);
	ESP_LOGI(TAG,"gxBias = %d", calib.gxBias);
	ESP_LOGI(TAG,"gyBias = %d", calib.gyBias);
	ESP_LOGI(TAG,"gzBias = %d", calib.gzBias);
	ESP_LOGI(TAG,"mxBias = %d", calib.mxBias);
	ESP_LOGI(TAG,"myBias = %d", calib.myBias);
	ESP_LOGI(TAG,"mzBias = %d", calib.mzBias);
	ESP_LOGI(TAG,"mxSens = %d", calib.mxSens);
	ESP_LOGI(TAG,"mySens = %d", calib.mySens);
	ESP_LOGI(TAG,"mzSens = %d", calib.mzSens);
#endif
   return 0;
   }

