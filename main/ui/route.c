#include "common.h"
#include "config.h"
#include "lcd7565.h"
#include "btn.h"
#include "ui.h"
#include "options.h"
#include "route.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "spiffs_vfs.h"

#define TAG "route"

#define CONTINUE() {ESP_LOGI(TAG,"error at line %d\r\n", __LINE__); continue;} 
//#define MIN(x,y) ((x) < (y) ? (x) : (y))

ROUTE Route;
ROUTE* pRoute = &Route;

#define MAX_ROUTES 7
int NumRoutes;
int RouteSel;
char RouteFileNames[MAX_ROUTES][20];

int rte_handleRouteSelEvent();
void rte_displayRouteSel();

int rte_selectRoute(){
   DIR *dir = NULL;
   struct dirent *ent;
   char tpath[100];
   struct stat sb;
#ifdef ROUTE_DEBUG
   ESP_LOGI(TAG,"LIST of DIR [/spiffs/]\r\n");
#endif
   dir = opendir("/spiffs/");
   NumRoutes = 0;
   if (dir) {
      while ((ent = readdir(dir)) != NULL) {
    	   sprintf(tpath, "/spiffs/");
         strcat(tpath, ent->d_name);
		   int res = stat(tpath, &sb);
			if ((res == 0) && (ent->d_type == DT_REG)) {
            if (strstr(tpath, ".wpt") || strstr(tpath,".WPT")){
               strcpy(RouteFileNames[NumRoutes], ent->d_name);
               ESP_LOGI(TAG,"Found route file %s", RouteFileNames[NumRoutes]);
               NumRoutes++;
               if (NumRoutes == MAX_ROUTES) break;
               }
			   }
         }
      }
   else {
      ESP_LOGE(TAG,"error opening /spiffs/");
      return -1;
      }
   if (NumRoutes == 0) return -1;

   RouteSel = 0; // default, do not use a route
   rte_displayRouteSel();
   btn_clear();
   while (rte_handleRouteSelEvent() == 0) {
      btn_debounce();
      delayMs(30);
      }
   if (RouteSel == 0) return -1;
   sprintf(tpath, "/spiffs/");
   strcat(tpath, RouteFileNames[RouteSel-1]);
   return rte_loadRoute(tpath);
   }


void rte_displayRouteSel() {
  lcd_clear();
  // BtnR to traverse list, Btn0 to select
  lcd_printlnf(false, 0, "R->change   0->sel"); 
  lcd_printlnf(false, 1, "%c no route", RouteSel == 0 ? '*' : ' ');
  for (int inx = 0; inx < NumRoutes; inx++) {
      lcd_printlnf(false, 2+inx, "%c %s", RouteSel == 1+inx ? '*' : ' ', RouteFileNames[inx]);
      }
  lcd_sendFrame();
  }

#define RTE_IDLE_COUNT 330 // ~10 seconds at 30mS debounce interval

int rte_handleRouteSelEvent() {
	static int countDown = RTE_IDLE_COUNT;

	if (BtnRPressed) {
		btn_clear();
		countDown = RTE_IDLE_COUNT;
		RouteSel++;
		if (RouteSel > NumRoutes) {
			RouteSel = 0;
         	}
		rte_displayRouteSel();
		return 0;
      	}
	else
	if (Btn0Pressed || (countDown <= 0)) {
		btn_clear();
		return 1;
      	}
   else {
	   countDown--;
	   return 0;
   	   }
   }

// expects FormatGEO waypoint text file ".wpt" as output by xcplanner (xcplanner.appspot.com)
// You can edit the waypoint text file to add a waypoint radius in meters at the end of  each
// waypoint entry line. If a waypoint radius is not found, the default waypoint radius
// (as specified in options.txt) will be used.

int rte_loadRoute(char* szFileName) {
   FILE * fwpt = NULL;
   int cnt, maxCnt;
   float lat,lon,alt,sec,radius;
   int32_t deg,min;
   char* szToken;
   char szLine[100];
   char c;

   pRoute->numWpts = 0;
   pRoute->nextWptInx = 0;
   if ((fwpt = fopen( szFileName, "r" )) == NULL){
      ESP_LOGI(TAG, "error opening file %s\r\n", szFileName);
      return -1;
      }
   rte_readLine(fwpt,szLine);
	szToken = strtok(szLine," \r\n");
   if (strcmp(szToken, "$FormatGEO") != 0) {
      ESP_LOGI(TAG,"incorrect format\r\n");
      return -2;
      }

   // malformed lines are skipped, so check the total number of waypoints is as expected.
   // waypoints without a radius are allowed, then the default radius is used. So xcplanner output
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
      pRoute->wpt[pRoute->numWpts].latdeg = lat;

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
      pRoute->wpt[pRoute->numWpts].londeg = lon;

      szToken = strtok(NULL," \t\r\n");
      if (szToken == NULL) CONTINUE();
      if (sscanf(szToken,"%f",&alt) != 1) CONTINUE();
      pRoute->wpt[pRoute->numWpts].altm = alt;

      szToken = strtok(NULL," \t\r\n");
      if (szToken == NULL) { // no radius found, use radius value from options.txt
         pRoute->wpt[pRoute->numWpts].radiusm = opt.misc.waypointRadiusm;
         pRoute->numWpts++;
         continue; 
         }
      if (sscanf(szToken,"%f",&radius) != 1) CONTINUE();
      pRoute->wpt[pRoute->numWpts].radiusm = radius;
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

int32_t rte_totalDistance() {
   int32_t routeDistancem = 0;
   for (int winx = 0; winx < pRoute->numWpts; winx++) {
      if (winx < pRoute->numWpts-1) {
         routeDistancem += gps_haversineDistancem(pRoute->wpt[winx].latdeg, pRoute->wpt[winx].londeg,
                              pRoute->wpt[winx+1].latdeg, pRoute->wpt[winx+1].londeg);
         }
      }
   return routeDistancem;
   }

