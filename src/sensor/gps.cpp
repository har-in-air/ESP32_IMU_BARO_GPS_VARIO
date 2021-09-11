#include "common.h"
#include <math.h>
#include <driver/uart.h>
#include "config.h"
#include "drv/cct.h"
#include "nv/flashlog.h"
#include "nv/options.h"
#include "ui/ui.h"
#include "sensor/gps.h"

static const char* TAG = "gps";

static bool ubx_detectUBX(int baudrate);
static bool ubx_detectNMEA(int baudrate);

const uint8_t GpsUBXNAVPVTHdr[] = {0xB5,0x62,0x01,0x07, 0x5C, 0x00};

static int  NumValidHdrBytes;
static int  GpsState;
static int  PktReceivedBytes;

volatile NAV_PVT NavPvt;
volatile bool IsGpsNavUpdated;
									   
static uart_port_t UartNum;
static uint8_t UartRcvBuffer[UART_RX_BUFFER_SIZE];

#if 0
static void ubx_checksum(uint8_t* buffer, int numbytes, uint8_t* pcka, uint8_t* pckb) {
   uint8_t cka, ckb;
   cka = ckb = 0;
   for (int inx = 0; inx < numbytes; inx++) {
      cka += buffer[inx];
      ckb += cka;
      }
   *pcka = cka;
   *pckb = ckb;
   }
#endif

const uint8_t CfgGNSS[] = 
{0xB5,0x62,0x06,0x3E,0x3C,0x00,0x00,0x00,0x20,0x07,0x00,0x08,0x20,0x00,0x01,
0x00,0x01,0x01,0x01,0x01,0x03,0x00,0x00,0x00,0x01,0x01,0x02,0x04,0x08,0x00,
0x00,0x00,0x01,0x01,0x03,0x08,0x10,0x00,0x00,0x00,0x01,0x01,0x04,0x00,0x08,
0x00,0x00,0x00,0x01,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x06,0x08,
0x0E,0x00,0x00,0x00,0x01,0x01,0x3C,0xAD};

/* pedestrian mode
const uint8_t CfgNAV5[] = 
{0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x08,0x02,0x00,0x00,0x00,0x00,0x10,
0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x5E,0x01,0x00,0x3C,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x85,0x2B};
*/

// aircraft < 4G, utc gps
const uint8_t CfgNAV5[] = 
{0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x02, 
0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 
0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x51, 0x10};


const uint8_t CfgPRT[] = 
{0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,
0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};

const uint8_t CfgRATE[] = 
{0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};

const uint8_t CfgCFG[] = 
{0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,
0x00,0x00,0x00,0x01,0x1B,0xA9};

const uint8_t CfgDisableGGA[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x23}; 

const uint8_t CfgDisableGLL[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};  

const uint8_t CfgDisableGSV[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38}; 

const uint8_t CfgDisableRMC[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x3F};

const uint8_t CfgDisableVTG[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46};

const uint8_t CfgDisableGSA[] = 
{0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31}; 

const uint8_t CfgEnableNAVPVT[] =
{0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};


static void ubx_config9600() {
   uart_set_baudrate(UartNum, 9600);
   delayMs(10); 
   uart_write_bytes(UartNum, (const char*)CfgPRT, sizeof(CfgPRT));
   uart_wait_tx_done(UartNum, 100/portTICK_RATE_MS);
   uart_set_baudrate(UartNum, 115200); 
   delayMs(10);
   }


static void ubx_config115200() {
   uart_set_baudrate(UartNum, 115200); 
   delayMs(10);
   uart_write_bytes(UartNum, (const char*)CfgDisableGGA, sizeof(CfgDisableGGA));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgDisableGLL, sizeof(CfgDisableGLL));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgDisableGSV, sizeof(CfgDisableGSV));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgDisableRMC, sizeof(CfgDisableRMC));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgDisableVTG, sizeof(CfgDisableVTG));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgDisableGSA, sizeof(CfgDisableGSA));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgEnableNAVPVT, sizeof(CfgEnableNAVPVT));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgGNSS, sizeof(CfgGNSS));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgNAV5, sizeof(CfgNAV5));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgRATE, sizeof(CfgRATE));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   uart_write_bytes(UartNum, (const char*)CfgCFG, sizeof(CfgCFG));
   uart_wait_tx_done(UartNum, 50/portTICK_RATE_MS);
   }


const unsigned char szNMEA_GNGLL[] = "$GNGLL,";
const unsigned char szNMEA_GPGLL[] = "$GPGLL,";

static bool ubx_detectNMEA(int baudrate) {
   uart_set_baudrate(UartNum, baudrate); 
	int cnt = 0;
   uint32_t marker = cct_setMarker();
	do {
      uint8_t b;
      uart_read_bytes(UartNum, &b, 1, 20 / portTICK_RATE_MS);
   	if (b == szNMEA_GNGLL[cnt])   {
    		cnt++;
   		}
   	else {
  			cnt = 0;
  			}  
		} while ((cnt < 7) && (cct_elapsedUs(marker) < 2000000)); // wait at least 2 second
   if (cnt == 7) return true;

	marker = cct_setMarker();
   cnt = 0;
	do {
      uint8_t b;
      uart_read_bytes(UartNum, &b, 1, 20 / portTICK_RATE_MS);
   	if (b == szNMEA_GPGLL[cnt])   {
    		cnt++;
   		}
   	else {
  			cnt = 0;
  			}  
		} while ((cnt < 7) && (cct_elapsedUs(marker) < 2000000)); // wait at least 2 second
   if (cnt == 7) return true;
   return false;
	}


const uint8_t ubxnavpvthdr[] = {0xB5, 0x62, 0x01, 0x07};

static bool ubx_detectUBX(int baudrate) {
   uart_set_baudrate(UartNum, baudrate); 
	int cnt = 0;
   uint32_t marker = cct_setMarker();
	do {
      uint8_t b;
      uart_read_bytes(UartNum, &b, 1, 20 / portTICK_RATE_MS);
   	if (b == ubxnavpvthdr[cnt])   {
    		cnt++;
   		}
   	else {
  			cnt = 0;
  			}  
		} while ((cnt < 4) && (cct_elapsedUs(marker) < 2000000)); // wait at least 2 seconds

	return (cnt < 4) ? false : true;
	}


bool gps_config() {
   esp_err_t err;
   UartNum = GPS_UART_NUM;
   uart_config_t uart_config = {
     .baud_rate = 115200,
     .data_bits = UART_DATA_8_BITS,
     .parity = UART_PARITY_DISABLE,
     .stop_bits = UART_STOP_BITS_1,
     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
   };
   err = uart_param_config(UartNum, &uart_config);
   if (err != ESP_OK) {
      ESP_LOGE(TAG, "error configuring uart params");
      return false;
      }
   err = uart_set_pin(UartNum, pinGpsTXD, pinGpsRXD, pinGpsRTS, pinGpsCTS);
   if (err != ESP_OK) {
      ESP_LOGE(TAG, "error configuring uart pins");
      return false;
      }
   err = uart_driver_install(UartNum, UART_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
   if (err != ESP_OK) {
      ESP_LOGE(TAG, "error installing uart driver");
      return false;
      }

   bool nmea9600 = false;
   bool ubx9600 = false; 
   bool nmea115200 = false;
   bool ubx115200 = false; 

	ESP_LOGI(TAG, "Testing UBX 115200");
	ubx115200 = ubx_detectUBX(115200);
	if (ubx115200) {
		ESP_LOGI(TAG, "Found UBX packet at 115200 baud");
		}
	else {
   		ESP_LOGI(TAG, "Testing NMEA 9600");
   		nmea9600 = ubx_detectNMEA(9600);
   		if (nmea9600) {
      		ESP_LOGI(TAG, "Found NMEA packet at 9600 baud");
      		}   
   		else {
      		ESP_LOGI(TAG, "Testing UBX 9600");
      		ubx9600 = ubx_detectUBX(9600);
      		if (ubx9600) {
         		ESP_LOGI(TAG, "Found UBX packet at 9600 baud");
         		}   
         	else {
            	ESP_LOGI(TAG, "Testing NMEA 115200");
            	nmea115200 = ubx_detectNMEA(115200);
            	if (nmea115200) {
               		ESP_LOGI(TAG, "Found NMEA packet at 115200 baud");
               		}   
            	else{ 
               		ESP_LOGE(TAG, "Could not detect protocol");
               		return false;
               		}
            	}
         	} 
      	}    

   if (nmea9600 || ubx9600){
      ESP_LOGI(TAG, "configure ubx port 9600->115200");
      ubx_config9600();
      delayMs(1000);
       ESP_LOGI(TAG, "Testing NMEA 115200");
       nmea115200 = ubx_detectNMEA(115200);
       if (nmea115200) {
          ESP_LOGI(TAG, "Found NMEA packet at 115200 baud");
          }   
      }
   	ESP_LOGI(TAG, "configuring @115200baud");
   	ubx_config115200();

	NumValidHdrBytes = 0;
	PktReceivedBytes = 0;
	IsGpsNavUpdated = false;
	GpsState = GPS_STATE_IDLE;
	return true;
	}


int gps_packetChecksum(uint8_t* pBuf, int numBytes) {
	return 1;
	}


void gps_stateMachine()  {
   int len = uart_read_bytes(UartNum, UartRcvBuffer, UART_RX_BUFFER_SIZE, 20 / portTICK_RATE_MS);
   if (len) {
	   for (int inx = 0; inx < len; inx++) { 
         uint8_t rcvb = UartRcvBuffer[inx];
		   switch(GpsState) {
	         case GPS_STATE_IDLE :
	         default :
	   		if (rcvb == GpsUBXNAVPVTHdr[NumValidHdrBytes]) {
	    		   NumValidHdrBytes++;
	   			}
	   		else {
	  			   NumValidHdrBytes = 0; // reset 
	  			   }  
	   		if (NumValidHdrBytes == 6)	{ // header found
				   NumValidHdrBytes = 0;			
				   PktReceivedBytes = 0;
			      GpsState = GPS_STATE_PKTRCV;
					}
	        break;
	        			
			
	        case GPS_STATE_PKTRCV :
	        NavPvt.pktBuffer[PktReceivedBytes++] = rcvb;
			  if (PktReceivedBytes == NAV_PVT_PKT_NUM_BYTES) {
				   if (!gps_packetChecksum((uint8_t*)NavPvt.pktBuffer, NAV_PVT_PKT_NUM_BYTES )) {
	               PktReceivedBytes = 0;
	               GpsState = GPS_STATE_IDLE;
					   ESP_LOGE(TAG,"packet checksum error");
	               }
				   else {
                  gps_updateFlashLogRecord();        
                  IsGpsNavUpdated = 1;
                  //ESP_LOGD(TAG, "NAV_PVT tow %d",NavPvt.timeOfWeekmS);
	               PktReceivedBytes = 0;
	           	   GpsState = GPS_STATE_IDLE;
					   }
				   }            
	        break;					   
	        }
         }
      }
   } 


void gps_updateFlashLogRecord() {
   static int counter = 0;
	if ((opt.misc.logType == LOGTYPE_IBG) && FlashLogMutex) {
		if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
		   FlashLogIBGRecord.hdr.gpsFlags = NavPvt.nav.fixType;
		   FlashLogIBGRecord.gps.timeOfWeekmS = NavPvt.nav.timeOfWeekmS;
		   FlashLogIBGRecord.gps.heightMSLmm = NavPvt.nav.heightMSLmm;
		   FlashLogIBGRecord.gps.vertAccuracymm = NavPvt.nav.vertAccuracymm;
		   FlashLogIBGRecord.gps.velNorthmmps = NavPvt.nav.velNorthmmps;
		   FlashLogIBGRecord.gps.velEastmmps = NavPvt.nav.velEastmmps;
		   FlashLogIBGRecord.gps.velDownmmps = NavPvt.nav.velDownmmps;
		   FlashLogIBGRecord.gps.velAccuracymmps = NavPvt.nav.speedAccuracymmps;
		   FlashLogIBGRecord.gps.lonDeg7 = NavPvt.nav.lonDeg7;
		   FlashLogIBGRecord.gps.latDeg7 = NavPvt.nav.latDeg7;	
         }
		xSemaphoreGive( FlashLogMutex );
		}
   else 
	if ((opt.misc.logType == LOGTYPE_GPS) && IsGpsTrackActive) {
      counter++;// 100mS gps fix interval 
      if (counter >= (opt.misc.trackIntervalSecs*10)) {
         counter = 0;
         FlashLogGPSRecord.hdr.magic =FLASHLOG_GPS_MAGIC;
         FlashLogGPSRecord.hdr.fixType = NavPvt.nav.fixType;
         FlashLogGPSRecord.hdr.numSV = NavPvt.nav.numSV;
	      FlashLogGPSRecord.trkpt.posDOP = NavPvt.nav.posDOP;	
	      FlashLogGPSRecord.trkpt.utcYear = NavPvt.nav.utcYear;
	      FlashLogGPSRecord.trkpt.utcMonth = NavPvt.nav.utcMonth;
	      FlashLogGPSRecord.trkpt.utcDay = NavPvt.nav.utcDay;	
	      FlashLogGPSRecord.trkpt.utcHour = NavPvt.nav.utcHour;
	      FlashLogGPSRecord.trkpt.utcMinute = NavPvt.nav.utcMinute;
	      FlashLogGPSRecord.trkpt.utcSecond = NavPvt.nav.utcSecond;	
	      FlashLogGPSRecord.trkpt.nanoSeconds = NavPvt.nav.nanoSeconds;	
	      FlashLogGPSRecord.trkpt.heightMSLmm = NavPvt.nav.heightMSLmm;
	      FlashLogGPSRecord.trkpt.lonDeg7 =  NavPvt.nav.lonDeg7;
	      FlashLogGPSRecord.trkpt.latDeg7 =  NavPvt.nav.latDeg7;	
	      flashlog_writeGPSRecord(&FlashLogGPSRecord); 
         }
		}		
	}



// great-circle distance using WGS-84 average earth radius
int32_t gps_haversineDistancem(float lat1deg, float lon1deg, float lat2deg, float lon2deg)  {
   float  dlatrad, dlonrad, lat1rad, lat2rad, sindlat, sindlon, a, c, distancem;

	dlatrad = (lat2deg - lat1deg)*PI_DIV_180;
	dlonrad = (lon2deg - lon1deg)*PI_DIV_180;

   lat1rad = lat1deg*PI_DIV_180;
   lat2rad = lat2deg*PI_DIV_180;

   sindlat = sin(dlatrad/2.0f);
   sindlon = sin(dlonrad/2.0f);

   a = sindlat * sindlat + cos(lat1rad) * cos(lat2rad) * sindlon * sindlon;
   c = 2 * atan2(sqrt(a), sqrt(1-a));
   distancem = 6371009.0f * c;  // average earth radius in m (WGS-84)
   return (int32_t)(distancem + 0.5f);
   }

 
int32_t gps_bearingDeg(float lat1, float lon1, float lat2, float lon2) {
	float lat1rad,lat2rad,dlonrad,x,y,b;
	int32_t bearing;
	
	dlonrad = (lon2 - lon1)*PI_DIV_180;
	lat1rad = lat1*PI_DIV_180;
	lat2rad = lat2*PI_DIV_180;
	
	x =  cos(lat1rad)*sin(lat2rad) - sin(lat1rad)*cos(lat2rad)*cos(dlonrad);
	y = sin(dlonrad)*cos(lat2rad);
	b = atan2(y,x);
	b += _2_PI;
	if (b >= _2_PI) b -= _2_PI; // convert to [0, 2*pi] radians
	b *= _180_DIV_PI;  // translate to [0,360] degrees
	bearing = (int32_t)(b + 0.5f);
	CLAMP(bearing,0,359);
	return bearing;
	}


void gps_localDateTime(NAV_PVT* pn, int* plYear, int* plMonth, int* plDay, int* plHour, int* plMinute) {
	*plYear = pn->nav.utcYear;
	*plMonth = pn->nav.utcMonth;
	*plDay = pn->nav.utcDay;
   int tmp = pn->nav.utcHour*60 + pn->nav.utcMinute + (int)opt.misc.utcOffsetMins; 
	if (tmp > 1440) {
		tmp -= 1440;
		*plDay += 1;
		switch (*plMonth) {
			case 1 : 
			case 3 :
			case 5 :
			case 7 :
			case 8 :
			case 10 :
			case 12 : if (*plDay > 31) {
							*plDay = 1; 
							*plMonth += 1;
							} 
						break;				
			case 4 :
			case 6 :
			case 9 :
			case 11 : if (*plDay > 30) {
							*plDay = 1; 
							*plMonth += 1;
							}
						break;						
			case 2 : if ((*plYear%4) == 0) {
						   if (*plDay > 29) {*plDay = 1; *plMonth += 1;}
						   }
						else {
							if (*plDay > 28) {*plDay = 1; *plMonth += 1;}
							}
						break;
						}

		   if (*plMonth > 12) {*plMonth = 1; *plYear += 1;
			   }
		   }
		else 
      if (tmp < 0) {
			tmp += 1440;
			*plDay -= 1;
			switch (*plMonth) {
				case 1 : 
				case 2 :
				case 4 :
				case 6 : 
				case 8 :
				case 9 :
				case 11 : if (*plDay < 1) {*plDay = 31; *plMonth -= 1;} 
						break;
				
				case 5 :
				case 7 :
				case 10 :
				case 12 : if (*plDay < 1) {*plDay = 30; *plMonth -= 1;}
						break;
						
				case 3 : if ((*plYear%4) == 0) {
								if (*plDay < 1) {*plDay = 29; *plMonth -= 1;}
								}
							else {
								if (*plDay < 1) {*plDay = 28; *plMonth -= 1;}
								}
						break;
						}
					if (*plMonth < 1) {*plMonth = 12; *plYear -= 1;	}
					}
	*plHour = tmp/60;
	*plMinute = tmp%60;
   }

