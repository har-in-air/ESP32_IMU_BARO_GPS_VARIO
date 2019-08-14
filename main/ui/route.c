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

#define CONTINUE() {ESP_LOGE(TAG,"error at line %d", __LINE__); continue;}

ROUTE Route;
ROUTE* pRoute = &Route;

#define MAX_ROUTES 7

#define MAX_FILENAME_LENGTH	20

int NumRoutes;
int RouteSel;
char RouteFileNames[MAX_ROUTES][MAX_FILENAME_LENGTH+1];

static bool rte_handleRouteSelEvent();
static bool rte_readLine(FILE* pFile, char* szBuf);
static void rte_displayRouteSel();
static bool rte_loadRoute(char* szFileName);


bool rte_selectRoute(){
	DIR *dir = NULL;
	struct dirent *ent;
	char tpath[100];
	struct stat sb;
#ifdef ROUTE_DEBUG
	ESP_LOGD(TAG,"LIST of DIR [/spiffs/]\r\n");
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
					if (strlen(ent->d_name) <= MAX_FILENAME_LENGTH ){
						strcpy(RouteFileNames[NumRoutes], ent->d_name);
						ESP_LOGD(TAG,"Found route file %s", RouteFileNames[NumRoutes]);
						NumRoutes++;
						if (NumRoutes == MAX_ROUTES) break;
						}
					else {
						ESP_LOGE(TAG,"filename > %d chars", MAX_FILENAME_LENGTH);
						}
					}
				}
         	}
      	}
	else {
		ESP_LOGE(TAG,"error opening /spiffs/");
		return false;
      	}
	if (NumRoutes == 0) return false;

	RouteSel = 0; // default : do not use a route
	rte_displayRouteSel();
	btn_clear();
	while (!rte_handleRouteSelEvent()) {
		btn_debounce();
		delayMs(30);
      	}
	if (RouteSel == 0) return false;
	sprintf(tpath, "/spiffs/");
	strcat(tpath, RouteFileNames[RouteSel-1]);
	return rte_loadRoute(tpath);
   	}


static void rte_displayRouteSel() {
  lcd_clear();
  lcd_printlnf(false, 0, "%cNo route", RouteSel == 0 ? '*' : ' ');
  for (int inx = 0; inx < NumRoutes; inx++) {
      lcd_printlnf(false, 1+inx, "%c%s", RouteSel == 1+inx ? '*' : ' ', RouteFileNames[inx]);
      }
  lcd_sendFrame();
  }

#define RTE_IDLE_COUNT 330 // ~10 seconds at 30mS debounce interval (if you do nothing)

static bool rte_handleRouteSelEvent() {
	static int countDown = RTE_IDLE_COUNT;

	if (BtnLPressed) {
		btn_clear();
		countDown = RTE_IDLE_COUNT;
		if (RouteSel > 0) RouteSel--;
		rte_displayRouteSel();
		return false;
      	}
	if (BtnRPressed) {
		btn_clear();
		countDown = RTE_IDLE_COUNT;
		if (RouteSel < NumRoutes+1)	RouteSel++;
		rte_displayRouteSel();
		return false;
   		}
	else
	if (Btn0Pressed || (countDown <= 0)) {
		btn_clear();
		return true;
      	}
   else {
	   countDown--;
	   return false;
   	   }
   }


// expects FormatGEO waypoint text file ".wpt" as output by xcplanner (xcplanner.appspot.com)
// You can edit the waypoint text file to add a waypoint radius in meters at the end of  each
// waypoint entry line. If a waypoint radius is not found, the user-specified default waypoint radius
// (in options.txt) will be used.

static bool rte_loadRoute(char* szFileName) {
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
      ESP_LOGE(TAG, "error opening file %s\r\n", szFileName);
      return false;
      }
   rte_readLine(fwpt,szLine);
	szToken = strtok(szLine," \r\n");
   if (strcmp(szToken, "$FormatGEO") != 0) {
      ESP_LOGE(TAG,"incorrect format\r\n");
      return false;
      }

   // malformed lines are skipped, so check the total number of waypoints is as expected.
   // Waypoints without a radius are allowed, for these the waypoint radius from options.txt
   // is used. So xcplanner output files can be used without any modification if all the waypoints have the same radius.
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
    return true;
    }


static bool rte_readLine(FILE* pFile, char* szBuf) {
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
    return ((*szBuf) == 0 ? false : true);
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

