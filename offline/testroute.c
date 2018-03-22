#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "common.h"
#include "route.h"

#define MIN(x,y)                 ((x) < (y) ? (x) : (y))

ROUTE Route;
ROUTE* pRoute = &Route;

#define CONTINUE() {printf("error at line %d\r\n", __LINE__); continue;} 

int32_t gps_haversineDistancem(float lat1deg, float lon1deg, float lat2deg, float lon2deg);

// Expects FormatGEO formatted .wpt output files from xcplanner (xcplanner.appspot.com).
// Edit the file to add waypoint radius (in meters) to each waypoint. If the radius is not found, 
// a default user-configurable radius is used. 

int main(int argc, char* argv[]) {
   if (argc != 2) {
      printf("usage : %s <formatgeo.wpt>\r\n", argv[0]);
      return -1;
      }
   rte_loadRoute(argv[1]);

   int32_t routeDistance = 0;
   for (int winx = 0; winx < pRoute->numWpts; winx++) {
      printf("%d : ID %s lat %f lon %f alt %f radius %f\r\n",winx, pRoute->wpt[winx].szID, pRoute->wpt[winx].latdeg, pRoute->wpt[winx].londeg, pRoute->wpt[winx].altm, pRoute->wpt[winx].radiusm); 
      if (winx < pRoute->numWpts-1) routeDistance += gps_haversineDistancem(pRoute->wpt[winx].latdeg, pRoute->wpt[winx].londeg,
pRoute->wpt[winx+1].latdeg,pRoute->wpt[winx+1].londeg);
      }
   printf("Route distance : %.1fkm\r\n", ((float)routeDistance)/1000.0);
   return 0;
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
      printf("error opening file %s\r\n", szFileName);
      return -1;
      }
   rte_readLine(fwpt,szLine);
	szToken = strtok(szLine," \r\n");
   if (strcmp(szToken, "$FormatGEO") != 0) {
      printf("incorrect format\r\n");
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
      if (szToken == NULL) {
         pRoute->wpt[pRoute->numWpts].radiusm = WAYPT_RADIUS_DFLT;
         pRoute->numWpts++;
         CONTINUE();
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


// great-circle distance using WGS-84 average earth radius
int32_t gps_haversineDistancem(float lat1deg, float lon1deg, float lat2deg, float lon2deg)  {
   float  dlatrad, dlonrad, lat1rad, lat2rad, sindlat, sindlon, a, c, distancem;

	dlatrad = ((float)(lat2deg - lat1deg))*PI_DIV_180;
	dlonrad = ((float)(lon2deg - lon1deg))*PI_DIV_180;

   lat1rad = ((float)lat1deg)*PI_DIV_180;
   lat2rad = ((float)lat2deg)*PI_DIV_180;

   sindlat = sin(dlatrad/2.0f);
   sindlon = sin(dlonrad/2.0f);

   a = sindlat * sindlat + cos(lat1rad) * cos(lat2rad) * sindlon * sindlon;
   c = 2 * atan2(sqrt(a), sqrt(1-a));
   distancem = 6371009.0f * c;  // average earth radius in m (WGS-84)
   return (int32_t)(distancem + 0.5f);
   }

