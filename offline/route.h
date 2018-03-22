#ifndef ROUTE_H_
#define ROUTE_H_

#define MAX_WAYPOINTS			100

#define MAX_ID_CHARS    10

#define WAYPT_RADIUS_MIN		5
#define WAYPT_RADIUS_MAX		99999
#define WAYPT_RADIUS_DFLT		50

typedef struct WAYPT_ {
    char szID[MAX_ID_CHARS];
    float latdeg;
    float londeg;
    float altm;
    float radiusm;
} WAYPT;

typedef struct ROUTE_ {
    int numWpts;
    WAYPT wpt[MAX_WAYPOINTS];
    }  ROUTE;


extern ROUTE* pRoute;

int rte_selectRoute();
int rte_readLine(FILE* pFile, char* szBuf);
int rte_loadRoute(char* szFileName);

#endif
