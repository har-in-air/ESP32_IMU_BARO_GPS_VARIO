#ifndef GPS_H_
#define GPS_H_


#define NAV_PVT_PKT_NUM_BYTES   	94
#define GPS_MAX_PKTS             4


#define GPS_PKT_FREE        1
#define GPS_PKT_RCV         2
#define GPS_PKT_RDY         3

#define GPS_STATE_IDLE        	11
#define GPS_STATE_PKTRCV        22

// ublox UBX binary protcol, NAV_PVT packet
// @ 10Hz and 115200Hz baudrate

/* example of binary stream after 3D fix

08:43:05  0000  0D 0A                                            ...
          
08:43:05  0000  B5 62 01 07 5C 00 7C FB 9E 1B E1 07 0B 03 08 2B  µb..\.|û..á....+
          0010  05 37 0B 00 00 00 76 1F 0A FA 03 01 00 0D 95 25  .7....v..ú.....%
          0020  4A 2E F7 37 BE 07 36 99 0C 00 36 99 0C 00 8D 09  J.÷7¾.6...6.....
          0030  00 00 CD 0D 00 00 64 00 00 00 FE FF FF FF 00 00  ..Í...d...þÿÿÿ..
          0040  00 00 64 00 00 00 00 00 00 00 8B 00 00 00 B0 CC  ..d...........°Ì
          0050  00 00 A4 00 00 00 00 00 00 00 00 00 00 00 00 00  ..¤.............
          0060  00 00 67 03    
*/		  

#define VALID_DATE	0x01
#define VALID_TIME	0x02
#define VALID_UTC	   0x04
#define VALID_MAG	   0x08


#define FIX_NONE			   0
#define FIX_DEAD_RECKONING	1
#define FIX_2D			   	2
#define FIX_3D				   3
#define FIX_GNSS_DR			4
#define FIX_TIME			   5

#define FLAGS1_GNSS_FIX_OK		0x01
#define FLAGS1_DIFF_SOLN		0x02
#define FLAGS1_HEADING_VALID	0x10


typedef struct NAV_PVT_PKT_ {
	uint32_t  	timeOfWeekmS;
	uint16_t	   utcYear;
	uint8_t		utcMonth;
	uint8_t		utcDay;
	uint8_t		utcHour;
	uint8_t		utcMinute;
	uint8_t		utcSecond;
	uint8_t		validity;
	uint32_t	   timeAccuracyNs;
	int32_t		nanoSeconds;
	uint8_t		fixType;
	uint8_t		flags1;
	uint8_t		flags2;
	uint8_t		numSV;
	int32_t 	   lonDeg7;
	int32_t 	   latDeg7;
	int32_t		heightEllipsoidmm;
	int32_t 	   heightMSLmm;
	uint32_t	   horzAccuracymm;
	uint32_t	   vertAccuracymm;
	int32_t		velNorthmmps;
	int32_t		velEastmmps;
	int32_t		velDownmmps;
	int32_t		groundSpeedmmps;
	int32_t		headingMotionDeg5;
	uint32_t	   speedAccuracymmps;
	uint32_t	   headingAccuracyDeg5;
	uint16_t	   posDOP;
	uint8_t		reserved;
	int32_t		headingDeg5;
	int16_t		magneticDeclinationDeg2;
	uint16_t	   declinationAccuracyDeg2;
	uint8_t	   ckA;
	uint8_t     ckB;
} NAV_PVT_PKT;

typedef union NAV_PVT_ {
   NAV_PVT_PKT nav;
   uint8_t     pktBuffer[NAV_PVT_PKT_NUM_BYTES];
} NAV_PVT;

extern volatile NAV_PVT NavPvt;
extern volatile bool 	IsGpsNavUpdated;
						
int  gps_packetChecksum(uint8_t* pBuf, int numBytes);
void gps_stateMachine();
int  gps_config(void);
void gps_updateFlashLogRecord();

int32_t gps_haversineDistancem(float lat1, float lon1, float lat2, float lon2);
int32_t gps_bearingDeg(float lat1, float lon1, float lat2, float lon2);
void gps_localDateTime(NAV_PVT* pn, int* plYear, int* plMonth, int* plDay, int* plHour, int* plMinute);

#endif

