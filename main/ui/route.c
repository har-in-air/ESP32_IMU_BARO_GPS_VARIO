#include "common.h"
#include "config.h"
#include "ui.h"
#include "route.h"

#define TAG "route"

#define CONTINUE() {ESP_LOGI(TAG,"error at line %d\r\n", __LINE__); continue;} 
#define MIN(x,y) ((x) < (y) ? (x) : (y))

ROUTE Route;
ROUTE* pRoute = &Route;

void rte_init(void) {
    }


int rte_eraseFile(char* szFileName) {
    return 1;
    }


int rte_loadRoute(char* szFileName) {
   FILE * fwpt = NULL;
   int cnt, maxCnt;
   float lat,lon,alt,sec,radius;
   int32_t deg,min;
   char* szToken;
   char szLine[100];
   char c;

   pRoute->numWpts = 0;
   if ((fwpt = fopen( szFileName, "r" )) == NULL){
      //lcd_clear();
      //lcd_printf(true,0,0,"%s not found",szFileName);
      //delayMs(1000);
      ESP_LOGI(TAG, "error opening file %s\r\n", szFileName);
      return -1;
      }
   rte_readLine(fwpt,szLine);
	szToken = strtok(szLine," \r\n");
   if (strcmp(szToken, "$FormatGEO") != 0) {
      //lcd_clear();
      //lcd_printf(true,0,0,"Format GEO not found");
      //delayMs(1000);
      ESP_LOGI(TAG,"incorrect format\r\n");
      return -2;
      }

   // malformed lines are skipped, so check the total number of waypoints is as expected.
   // waypoints without a radius are allowed, the default radius is used. So xcplanner output
   // files can be used without any modification.
   while (rte_readLine(fwpt,szLine)) {
	   szToken = strtok(szLine," ");
	   if (szToken == NULL) CONTINUE();

      maxCnt = MIN(MAX_ID_CHARS-1, strlen(szToken));
      for (cnt = 0; cnt < maxCnt; cnt++) {
         pRoute->wpt[pRoute->numWpts].szID[cnt] = szToken[cnt];
         }
      pRoute->wpt[pRoute->numWpts].szID[maxCnt] = 0;

      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (strlen(szToken) != 1) CONTINUE();
      c = szToken[0];
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%d",&deg) != 1) CONTINUE();
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%d",&min) != 1) CONTINUE();
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%f",&sec) != 1) CONTINUE();
      lat = sec/3600.0f + ((float)min)/60.0f + (float)deg;
      if ((c == 'S') || (c == 's')) lat = -lat;
      pRoute->wpt[pRoute->numWpts].latDeg = lat;

      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (strlen(szToken) != 1) CONTINUE();
      c = szToken[0];
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%d",&deg) != 1) CONTINUE();
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%d",&min) != 1) CONTINUE();
      szToken = strtok(NULL," \t");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken, "%f",&sec) != 1) CONTINUE();
      lon = sec/3600.0f + ((float)min)/60.0f + (float)deg;
      if ((c == 'W') || (c == 'w')) lon = -lon;
      pRoute->wpt[pRoute->numWpts].lonDeg = lon;

      szToken = strtok(NULL," \t\r\n");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken,"%f",&alt) != 1) CONTINUE();
      pRoute->wpt[pRoute->numWpts].altM = alt;

      szToken = strtok(NULL," \t\r\n");
      if (szToken == NULL) {
         pRoute->wpt[pRoute->numWpts].radiusM = WAYPT_RADIUS_DFLT;
         pRoute->numWpts++;
         CONTINUE();
         }
      if (sscanf(szToken,"%f",&radius) != 1) CONTINUE();
      pRoute->wpt[pRoute->numWpts].radiusM = radius;
      pRoute->numWpts++;   
      }
    fclose(fwpt);
    return 0;
    }


int rte_readLine(FILE* pFile, char* szBuf) {
    uint8_t b;
    char* psz = szBuf;
    if (pFile == NULL) return 0;
    while (fread(&b,1,1,pFile) == 1)  {
        *psz++ = b;
        if (b == '\n') {
            break;
            }
        }
    *psz = 0;
    return ((*szBuf) == 0 ? 0 : 1);
    }


