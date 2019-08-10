#ifndef FLASHLOG_H_
#define FLASHLOG_H_

#include "spiflash.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASHLOG_RECORD_MAXBYTES		80
#define FLASHLOG_MAX_ADDR		      (FLASH_SIZE_BYTES - FLASHLOG_RECORD_MAXBYTES) 

// 128Mbits = 16777216 bytes

// with high speed IBG logging, 1 second logs
// 500*(I_RECORD + IBG_HDR), 50*B_RECORD, 10*G_RECORD
// 500*40 + 50*4 + 10*36 = 20560 bytes 
// We can log 16777216/20560 seconds = 13.6 minutes of IBG data

// with gps track logging at highest sampling rate (1 second interval)
// 1 second logs 29 bytes
// We can log 16777216/29 seconds = 160.7 hours of gps tracks

#define FLASHLOG_IBG_MAGIC  0xA55A
#define FLASHLOG_GPS_MAGIC  0x9043

typedef struct IBG_HDR_ {
	uint16_t	magic; 
   uint8_t  gpsFlags; 
   uint8_t  baroFlags;
} IBG_HDR;

// GPS_HDR is deliberately the same size as IBG_HDR so that we
// can separate IBG logs from GPS tracks in a binary flash dump
typedef struct GPS_HDR_ {
	uint16_t	magic; 
   uint8_t  fixType; 
   uint8_t  numSV;
} GPS_HDR;

typedef struct I_RECORD_ {
	float 	gxNEDdps;
	float 	gyNEDdps;
	float 	gzNEDdps;
	float 	axNEDmG;
	float 	ayNEDmG;
	float 	azNEDmG;
	float 	mxNED;
	float 	myNED;
	float 	mzNED;
   } I_RECORD;

typedef struct B_RECORD_ {
   int32_t  heightMSLcm;
} B_RECORD;

typedef struct G_RECORD_ {
	uint32_t timeOfWeekmS;
	int32_t 	lonDeg7;
	int32_t 	latDeg7;	
	int32_t	heightMSLmm;
	uint32_t	vertAccuracymm;
	int32_t	velNorthmmps;
	int32_t	velEastmmps;
	int32_t	velDownmmps;
	uint32_t	velAccuracymmps;
} G_RECORD;

// IBG record for high-speed IMU+BARO+GPS data logging 
// (fixed rate : 500Hz for IMU, 50Hz for Baro, 10Hz for GPS)
typedef struct FLASHLOG_IBG_RECORD_ {
   IBG_HDR   hdr;
   I_RECORD  imu;
   B_RECORD  baro;
   G_RECORD  gps;
} FLASHLOG_IBG_RECORD;
 

typedef struct T_RECORD_ {
   uint16_t posDOP;
	uint16_t	utcYear;
	uint8_t	utcMonth;
	uint8_t	utcDay;
	uint8_t	utcHour;
	uint8_t	utcMinute;
	uint8_t	utcSecond;
	int32_t	nanoSeconds;
	int32_t 	lonDeg7;
	int32_t 	latDeg7;	
	int32_t	heightMSLmm;
} T_RECORD;

// GPS record for gps track logging at user-configurable
// intervals from 1 to 60 seconds
typedef struct FLASHLOG_GPS_RECORD_ {
   GPS_HDR  hdr;
   T_RECORD trkpt;
} FLASHLOG_GPS_RECORD;

extern FLASHLOG_IBG_RECORD FlashLogIBGRecord;
extern FLASHLOG_GPS_RECORD FlashLogGPSRecord;
extern SemaphoreHandle_t   FlashLogMutex;
extern uint32_t            FlashLogFreeAddress;

int   flashlog_init(void);
int   flashlog_isEmpty(void);
void 	flashlog_erase(uint32_t untilAddress);
int 	flashlog_writeIBGRecord(FLASHLOG_IBG_RECORD* pRecord);
int 	flashlog_readIBGRecord(uint32_t addr, FLASHLOG_IBG_RECORD* pRecord);
uint32_t flashlog_getFreeAddress();
int   flashlog_getNumIBGRecords();
int 	flashlog_writeGPSRecord(FLASHLOG_GPS_RECORD* pRecord);

#ifdef __cplusplus
}
#endif

#endif


