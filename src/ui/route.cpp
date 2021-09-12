#include "common.h"
#include <FS.h>
#include <LITTLEFS.h>
#include "config.h"
#include "drv/btn.h"
#include "nv/options.h"
#include "lcd7565.h"
#include "ui.h"
#include "route.h"

static const char* TAG = "route";

#define CONTINUE() {ESP_LOGE(TAG,"error at line %d", __LINE__); continue;}

ROUTE Route;
ROUTE* pRoute = &Route;

#define MAX_ROUTES            7
#define MAX_FILENAME_LENGTH	20

int NumRoutes;
int RouteSel;
char RouteFileNames[MAX_ROUTES][MAX_FILENAME_LENGTH+1];

static bool rte_handleRouteSelEvent();
static void rte_displayRouteSel();
static bool rte_loadRoute(char* szFileName);


bool rte_selectRoute(){
	ESP_LOGD(TAG, "select route");
	File root = LITTLEFS.open("/");
	if(!root){
		ESP_LOGE(TAG, "− failed to open LITTLES directory /");
		return false;
		}
	if(!root.isDirectory()){
		ESP_LOGE(TAG, " − not a directory");
      root.close();
		return false;
		}
	NumRoutes = 0;
	File file = root.openNextFile();
	while (file) {
		if (file.isDirectory()) continue;
		String fname = "/";
      fname += file.name();
		if (fname.endsWith(".wpt")) {
			if (fname.length() <= MAX_FILENAME_LENGTH) {
				ESP_LOGD(TAG, "Route file %s found", fname.c_str());
				strcpy(RouteFileNames[NumRoutes], fname.c_str());
				NumRoutes++;
				if (NumRoutes == MAX_ROUTES) break;
				}
			else {
				ESP_LOGD(TAG,"Route filename > %d chars", MAX_FILENAME_LENGTH);
				}
			}
		if (fname.endsWith(".txt")) {
			ESP_LOGV(TAG, "txt file %s found", fname.c_str());
			}
		file.close();
		file = root.openNextFile();
		}
	root.close();

	if (NumRoutes == 0) return false;
	ESP_LOGD(TAG, "Number of routes found = %d", NumRoutes);
	RouteSel = 0; // default : do not use a route
	rte_displayRouteSel();
	btn_clear();
	while (!rte_handleRouteSelEvent()) {
		btn_debounce();
		delayMs(30);
		}
	if (RouteSel == 0) return false;
	ESP_LOGD(TAG, "Selected route file %s", RouteFileNames[RouteSel-1]);
	return rte_loadRoute(RouteFileNames[RouteSel-1]);
	}


static void rte_displayRouteSel() {
	lcd_clear_frame();
	lcd_printlnf(false, 0, "%cNo route", RouteSel == 0 ? '*' : ' ');
	for (int inx = 0; inx < NumRoutes; inx++) {
		lcd_printlnf(false, 1+inx, "%c%s", RouteSel == 1+inx ? '*' : ' ', RouteFileNames[inx]);
		}
	lcd_send_frame();
	}


#define RTE_IDLE_COUNT 200 // 5 seconds at 25mS debounce interval (if you do nothing)

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
// Each line in the waypoint file must be terminated with a carriage return and line feed (\r\n)

static bool rte_loadRoute(char* szFileName) {
   int cnt, maxCnt;
   float lat,lon,alt,sec,radius;
   int32_t deg,min;
   char* szToken;
   String sz;
   char* szLine;
   char c;

   pRoute->numWpts = 0;
   pRoute->nextWptInx = 0;
   File flrte = LITTLEFS.open(szFileName, FILE_READ);
   if (!flrte){
      ESP_LOGE(TAG, "error opening file %s", szFileName);
      return false;
      }
      
   if (!flrte.available()){
      ESP_LOGE(TAG, "Empty file %s", szFileName);
      return false;
      }
   sz = flrte.readStringUntil('\n');
   ESP_LOGV(TAG, "%s", sz.c_str());
   szLine = (char*)sz.c_str();
   szToken = strtok(szLine," \r\n");
   if (strcmp(szToken, "$FormatGEO") != 0) {
      ESP_LOGE(TAG,"No FormatGEO header found in file %s", szFileName);
      return false;
      }

   // malformed lines are skipped, so check the total number of waypoints is as expected.
   // Waypoints without a radius are allowed, for these the waypoint radius from options.txt
   // is used. So xcplanner output files can be used without any modification if all the waypoints have the same radius.
   while (flrte.available()) {
   	sz = flrte.readStringUntil('\n');
   	ESP_LOGV(TAG, "%s", sz.c_str());
      szLine = (char*)sz.c_str();

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
    flrte.close();
    return true;
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
