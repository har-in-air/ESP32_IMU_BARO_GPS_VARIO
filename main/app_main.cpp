#include <Arduino.h>
#include <WiFi.h>              
#include <ESP32WebServer.h>    // https://github.com/Pedroalbuquerque/ESP32WebServer
#include "server.h"

extern "C" {
#include "common.h"
#include "config.h"
#include <errno.h>
#include <sys/fcntl.h>
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "spiffs_vfs.h"
#include <ctype.h>
#include "driver/uart.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

#include "gps.h"
#include "lcd7565.h"
#include "cct.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "flashlog.h"
#include "spiflash.h"
#include "kalmanfilter3.h"
#include "lcd7565.h"
#include "audio.h"
#include "beeper.h"
#include "myadc.h"
#include "vspi.h"
#include "hspi.h"
#include "ringbuf.h"
#include "imu.h"
#include "calib.h"
#include "btn.h"
#include "ui.h"
#include "options.h"
}

#define TAG "main"

ESP32WebServer server(80);

volatile int LedState = 0;
volatile int BacklitCounter = 0;
volatile float kfAltitudeCm, kfClimbrateCps,iirClimbrateCps, glideRatio,glideRatioNew;
volatile float yawDeg, pitchDeg, rollDeg;

volatile SemaphoreHandle_t DrdySemaphore;
volatile bool DrdyFlag = false;

bool IsServer = false;

uint32_t clockPrevious;
uint32_t clockNow;
float 	imuTimeDeltaUSecs; // time between imu samples, in microseconds
float 	kfTimeDeltaUSecs; // time between kalman filter updates, in microseconds
 

void pinConfig() {	
	pinMode(pinLcdBklt, OUTPUT);
	LCD_BKLT_OFF();
   pinMode(pinLcdRST, OUTPUT);
	pinMode(pinLcdCS, OUTPUT);
   LCD_RST_HI();
	LCD_CS_HI();
	pinMode(pinLcdRS, OUTPUT);
	
	pinMode(pinFlashCS, OUTPUT);
	pinMode(pinImuCS, OUTPUT);
	pinMode(pinBaroCS, OUTPUT);
	FLASH_CS_HI();
	BARO_CS_HI();
	IMU_CS_HI();
	
   pinMode(pinAudioAmpEna, OUTPUT);
   AUDIO_AMP_DISABLE();

   pinMode(pinBtn0, INPUT); // external 10K pullup
   pinMode(pinBtnL, INPUT); // external 10K pullup
   pinMode(pinBtnM, INPUT); // external 10K pullup
   pinMode(pinBtnR, INPUT); // external 10K pullup
	
	pinMode(pinLED, OUTPUT);
	LED_OFF();

	pinMode(pinDRDYINT, INPUT); // external 10K pullup
	}


static void server_task(void *pvParameter){
  server.on("/",         server_homePage);
  server.on("/dir",      server_listDir);
  server.on("/upload",   server_fileUpload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, server_handleFileUpload);
  server.on("/download", server_fileDownload);
  server.on("/delete",   server_fileDelete);
  server.on("/datalog",  server_downloadDataLog);

  server.begin();
  while(1) {
     server.handleClient(); 
     delayMs(10); // yield to idle task
     }
  }


static void display_task(void *pvParameter) {
   int counter = 0;
   NAV_PVT navpvt;
   TRACK  track;
   IsGpsFixStable = false;
   IsLogging = false;
   IsTrackActive = false;
   IsLcdBkltEnabled = false;
   IsSpeakerEnabled = true;
   glideRatio = 1.0f;

	while(1) {
		if (IsGpsNavUpdated) {
			IsGpsNavUpdated = false;
         counter++;
         memcpy(&navpvt, (void*)&NavPvt, sizeof(NAV_PVT)); 
         float vn = (float)navpvt.nav.velNorthmmps;
         float ve = (float)navpvt.nav.velEastmmps;
         float horzVelmmps = sqrt(vn*vn + ve*ve);
         if (navpvt.nav.velDownmmps > 0) {
            glideRatioNew = horzVelmmps/navpvt.nav.velDownmmps;
            glideRatio = (glideRatio*(float)opt.misc.glideRatioIIR + glideRatioNew*(float)(100-opt.misc.glideRatioIIR))/100.0f;
            }

         if (counter >= 3) {
            counter = 0;
            // display update interval 0.3s
            if ((!IsGpsFixStable) && ((navpvt.nav.posDOP+50)/100 < opt.misc.gpsStableDOP)) {
               IsGpsFixStable = true;
               track.startLatDeg7 = navpvt.nav.latDeg7;
               track.startLonDeg7 = navpvt.nav.lonDeg7;
               track.startAltm = (navpvt.nav.heightMSLmm+500)/1000;
               }    
            if (IsGpsFixStable) {
               track.distanceFromStartm = gps_haversineDistancem(track.startLatDeg7,track.startLonDeg7,navpvt.nav.latDeg7,navpvt.nav.lonDeg7);
               if ((!IsTrackActive) && (track.distanceFromStartm >= opt.misc.trackStartThresholdm)) {
                  IsTrackActive = true;
                  track.startTowmS = navpvt.nav.timeOfWeekmS;
                  }
               }
            ui_updateFlightDisplay(&navpvt,&track);
            }
			}
		delayMs(10);
		}
	}	


static void gps_task(void *pvParameter) {
	ESP_LOGI(TAG, "gps task started");
   if (gps_config() < 0) {
      ESP_LOGE(TAG, "error configuring gps");
      while(1) {delayMs(100);}
      }
   while(1) {
      gps_stateMachine();
      }
   }  


void IRAM_ATTR drdyHandler(void) {
	DrdyFlag = true;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up imubaro_task immediately instead of on next FreeRTOS tick
		}
	//LED_TOGGLE();
	}


static void vario_taskConfig() {
   lcd_printlnf(true,0,"IMU config");
   if (mpu9250_config() < 0) {
      ESP_LOGE(TAG, "error MPU9250 config");
		lcd_printlnf(true,0,"IMU config failed");
      while (1) {delayMs(100);};
      }

   btn_clear();
   // if calib.txt was not found, accel and mag calibration is enforced automatically
   bool isAccelMagCalibRequired = !IsCalibrated;
   int counter = 300;
   while ((!isAccelMagCalibRequired) && counter--) {
   	lcd_printlnf(true,0,"Gyro calib in %ds",(counter+50)/100);
	   if (!BTN0()) {
         // manually force accel+mag calibration by pressing BTN0 during gyro calib countdown
         isAccelMagCalibRequired = true; 
         break;
         }
      delayMs(10);	
		}
   if (isAccelMagCalibRequired) {
      counter = 8;
      while (counter--) {
         lcd_printlnf(true,0,"Accel calib in %ds",counter+1);
         delayMs(1000);	
         }
      lcd_printf(true,0,0, "Calibrating Accel...");
      if (mpu9250_calibrateAccel() < 0) {
 	      lcd_printlnf(true,0,"Accel calib failed");
         while (1) {delayMs(100);};
         }
      delayMs(1000);
      counter = 5;
      while (counter--) {
         lcd_printlnf(true,0,"Mag calib in %ds",counter+1);
         delayMs(1000);	
         }
      lcd_printlnf(true,0,"Calibrating Mag...");
      if (mpu9250_calibrateMag()  < 0 ) {
 	      lcd_printlnf(true,0,"Mag calib failed");
         }
      }
   if (isAccelMagCalibRequired) {
      counter = 3;
      while (counter--) {
         lcd_printlnf(true,0,"Gyro calib in %ds",counter+1);
         }
      }
   lcd_printlnf(true,0,"Calibrating Gyro...");
   if (mpu9250_calibrateGyro() < 0) {
 	   lcd_printlnf(true,0,"Gyro calib failed");
      }
 	lcd_printlnf(true,0,"Baro config");
	if (ms5611_config() < 0) {
		ESP_LOGE(TAG, "error MS5611 config");
		lcd_printlnf(true,0, "Baro config failed");
		while (1) {delayMs(100);}
		}	

   ms5611_averagedSample(4);
   ESP_LOGI(TAG,"Altitude %dm Temperature %dC", (int)(ZCmAvg/100.0f), (int)CelsiusSample);
   lcd_printlnf(true,0,"ALT %dm TEMP %dC",(int)(ZCmAvg/100.0f), (int)CelsiusSample);
   kalmanFilter3_configure((float)opt.kf.zMeasVariance, 1000.0f*(float)opt.kf.accelVariance, KF_ACCELBIAS_VARIANCE, ZCmAvg, 0.0f, 0.0f);
   delayMs(2000);
   lcd_clear();
   lcd_printlnf(true,0,"GPS VARIO start ...");
	ms5611_initializeSampleStateMachine();
	vspi_setClockFreq(VSPI_CLK_HIGH_FREQHZ); // high clock frequency ok for sensor readout & flash writes
   beeper_config();
   ringbuf_init(); 
   }



static void vario_task(void *pvParameter) {
	float gxdps, gydps, gzdps, axmG, aymG, azmG, mx, my, mz;
   float gxNEDdps, gyNEDdps, gzNEDdps, axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED;

	ESP_LOGI(TAG, "vario task started");
   // setup time markers for imu, baro and kalman filter
	clockNow = clockPrevious = XTHAL_GET_CCOUNT();
   int drdyCounter = 0;   
   int baroCounter = 0;
   float kfTimeDeltaUSecs = 0.0f;
   float imuTimeDeltaUSecs;
	DrdySemaphore = xSemaphoreCreateBinary();
	attachInterrupt(pinDRDYINT, drdyHandler, RISING);

   while (1) {
  		xSemaphoreTake(DrdySemaphore, portMAX_DELAY);
	   clockNow = XTHAL_GET_CCOUNT();
      uint32_t marker =  cct_setMarker();
	   imuTimeDeltaUSecs = cct_intervalUs(clockPrevious, clockNow);
	   clockPrevious = clockNow;
      drdyCounter++;
      baroCounter++;
      mpu9250_getGyroAccelMagData( &gxdps, &gydps, &gzdps, &axmG, &aymG, &azmG, &mx, &my, &mz);
      axNEDmG = -aymG;
      ayNEDmG = -axmG;
      azNEDmG = azmG;
      gxNEDdps = gydps;
      gyNEDdps = gxdps;
      gzNEDdps = -gzdps;
      mxNED =  mx;
      myNED =  my;
      mzNED =  mz;
      float asqd = axNEDmG*axNEDmG + ayNEDmG*ayNEDmG + azNEDmG*azNEDmG;
      // constrain use of accelerometer data to the window [0.75G, 1.25G] for determining
      // the orientation quaternion
		int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
      int useMag = true;
		imu_mahonyAHRSupdate9DOF(useAccel, useMag,imuTimeDeltaUSecs/1000000.0f, DEG2RAD(gxNEDdps), DEG2RAD(gyNEDdps), DEG2RAD(gzNEDdps), axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED);
	   imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&yawDeg, (float*)&pitchDeg, (float*)&rollDeg);
      float gravityCompensatedAccel = imu_gravityCompensatedAccel(axNEDmG, ayNEDmG, azNEDmG, q0, q1, q2, q3);
      ringbuf_addSample(gravityCompensatedAccel); 
		kfTimeDeltaUSecs += imuTimeDeltaUSecs;

		if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			baroCounter = 0;     // alternating between pressure and temperature samples
         // one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = ms5611_sampleStateMachine(); 
			if ( zMeasurementAvailable ) { 
            // need average earth-z acceleration over the 20mS interval between z samples
            // z sample is from when pressure conversion was triggered, not read (i.e. 10mS ago). 
            // So we need to average the acceleration samples from the 20mS interval before that
				float zAccelAverage = ringbuf_averageOldestSamples(10); 
				kalmanFilter3_update(ZCmSample, zAccelAverage, ((float)kfTimeDeltaUSecs)/1000000.0f, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
            // use damped climbrate for lcd display and for glide ratio computation
            iirClimbrateCps = iirClimbrateCps*0.9f + 0.1f*kfClimbrateCps; 
            kfTimeDeltaUSecs = 0.0f;
				int32_t audioCps = INTEGER_ROUNDUP(kfClimbrateCps);
				if (IsSpeakerEnabled) {
               beeper_beep(audioCps);                
               }
			   if ((opt.misc.logType == LOGTYPE_IBG) && FlashLogMutex) {
			      if ( xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
                  FlashLogIBGRecord.hdr.baroFlags = 1;
				      FlashLogIBGRecord.baro.heightMSLcm = ZCmSample;
					   xSemaphoreGive( FlashLogMutex );
					   }
				   }
            }    
         }           
	   if ((opt.misc.logType == LOGTYPE_IBG) && FlashLogMutex) {
		   if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {      
            FlashLogIBGRecord.hdr.magic = FLASHLOG_IBG_MAGIC
;
			  	FlashLogIBGRecord.imu.gxNEDdps = gxNEDdps;
			   FlashLogIBGRecord.imu.gyNEDdps = gyNEDdps;
			   FlashLogIBGRecord.imu.gzNEDdps = gzNEDdps;
			   FlashLogIBGRecord.imu.axNEDmG = axNEDmG;
			   FlashLogIBGRecord.imu.ayNEDmG = ayNEDmG;
			   FlashLogIBGRecord.imu.azNEDmG = azNEDmG;
			   FlashLogIBGRecord.imu.mxNED = mxNED;
			   FlashLogIBGRecord.imu.myNED = myNED;
			   FlashLogIBGRecord.imu.mzNED = mzNED;
			   //LED_ON();
            // worst case imu+baro+gps record = 80bytes,
            //  ~130uS intra-page, ~210uS across page boundary
            if (IsLogging) {
               // out of memory, indicate in UI that logging has stopped
   	         if (flashlog_writeIBGRecord(&FlashLogIBGRecord) < 0) {
                  IsLogging = false;
                  } 
               memset(&FlashLogIBGRecord, 0, sizeof(FLASHLOG_IBG_RECORD));
               }
				//LED_OFF();
				xSemaphoreGive( FlashLogMutex );
				}			
			}  
      uint32_t eus = cct_elapsedUs(marker);
      if (drdyCounter >= 500) {
         drdyCounter = 0;
#ifdef IMU_DEBUG
         //ESP_LOGI(TAG,"%dus",eus);
			//ESP_LOGI(TAG,"\r\nY = %d P = %d R = %d", (int)yawDeg, (int)pitchDeg, (int)rollDeg);
			//ESP_LOGI(TAG,"ba = %d ka = %d v = %d",(int)ZCmSample, (int)kfAltitudeCm, (int)kfClimbrateCps);
#endif			

        // ESP_LOGI(TAG,"ax %.1f ay %.1f az %.1f", axmG, aymG, azmG);
        // ESP_LOGI(TAG,"gx %.1f gy %.1f gz %.1f", gxdps, gydps, gzdps);
        // ESP_LOGI(TAG,"mx %.1f my %.1f mz %.1f", mx, my, mz);
        // ESP_LOGI(TAG,"ax %.1f ay %.1f az %.1f", axNEDmG, ayNEDmG, azNEDmG);
        // ESP_LOGI(TAG,"gx %.1f gy %.1f gz %.1f", gxNEDdps, gyNEDdps, gzNEDdps);
        // ESP_LOGI(TAG,"mx %.1f my %.1f mz %.1f", mxNED, myNED, mzNED);

        // ESP_LOGI(TAG,"baro alt %dcm", (int)ZCmSample);
         //lcd_clear();
         //lcd_printf(0,0,"a %d %d %d", (int)axmG, (int)aymG, (int)azmG);
         //lcd_printf(1,0,"g %d %d %d", (int)gxdps, (int)gydps, (int)gzdps);
         //lcd_printf(2,0,"m %d %d %d", (int)mx, (int)my, (int)mz);
         //lcd_printf(0,0,"a %d %d %d", (int)axNEDmG, (int)ayNEDmG, (int)azNEDmG);
         //lcd_printf(1,0,"g %d %d %d", (int)gxNEDdps, (int)gyNEDdps, (int)gzNEDdps);
         //lcd_printf(2,0,"m %d %d %d", (int)mxNED, (int)myNED, (int)mzNED);
         }
      }
   }


extern "C" void app_main() {
   initArduino();
	ESP_LOGI(TAG, "esp32gpsvario compiled on %s at %s", __DATE__, __TIME__);
	pinConfig();

   // initialize SPIFFS file system and read calibration and configuration parameters
   vfs_spiffs_register();
   calib_init();
   opt_init();

   audio_config(pinAudioDAC);
   // turn off wifi radio to save power
   WiFi.mode(WIFI_OFF);

   // HSPI bus used for lcd interface
   hspi_config(pinHSCLK, pinHMOSI, pinHMISO, HSPI_CLK_FREQHZ);
	lcd_init();
   LCD_BKLT_ON();
   BacklitCounter = 330;
   uint32_t batteryVoltagemV = adc_batteryVoltage();
   ESP_LOGI(TAG, "battery voltage = %d.%03dV", batteryVoltagemV/1000, batteryVoltagemV%1000);
   lcd_printlnf(false,0,"%s %s", __DATE__, __TIME__);
   lcd_printlnf(true,1,"BAT %d.%03dV", batteryVoltagemV/1000, batteryVoltagemV%1000);

   // VSPI bus used for imu, baro and flash
   // start with low clock frequency for sensor configuration
   vspi_config(pinVSCLK, pinVMOSI, pinVMISO,VSPI_CLK_CONFIG_FREQHZ);
	if (flashlog_init() < 0) {
		ESP_LOGE(TAG, "Spi flash log error");
		lcd_printlnf(true,2,"Flash Error");		
		while (1) {delayMs(100);}
		}

	lcd_printlnf(true,2,"Flash %08d",  FlashLogFreeAddress);
   delayMs(2000);
   btn_clear();
   if (FlashLogFreeAddress) {
   	ESP_LOGI(TAG,"Press BTN0 within 3 seconds to erase flash");
	   LED_ON();
      bool isEraseRequired = false;
      int counter = 300;
      while (counter--) {
   	   lcd_printlnf(true,3,"BTN0 Erase %ds",(counter+50)/100);
	      if (!BTN0()) {
		      ESP_LOGI(TAG,"BTN0 PRESSED");
            isEraseRequired = true;
            break;
            }
         delayMs(10);	
		   }
      if (isEraseRequired) {
		   ESP_LOGI(TAG, "Erasing flash...");
		   lcd_printlnf(true,3,"Erasing...");
		   flashlog_erase();
		   ESP_LOGI(TAG, "Done");
		   lcd_printlnf(false,2,"Flash %08d", FlashLogFreeAddress);		
		   lcd_printlnf(true,3,"Flash Erased");
         }
      }
   delayMs(1000);
   IsServer = false;
	ESP_LOGI(TAG,"Press BTN0 within 3 seconds to start http server");
   int counter = 300;
   while (counter--) {
	   lcd_printlnf(true,3,"BTN0 Server %ds",(counter+50)/100);
      if (!BTN0()) {
	      ESP_LOGI(TAG,"BTN0");
	      IsServer = 1;
         break;
         }
      delayMs(10);
	   }

   if (IsServer) {
      lcd_clear();
      lcd_printlnf(false,0,"HTTP Server Mode");
      lcd_printlnf(false,1,"AP \"ESP32GpsVario\"");
      lcd_printlnf(false,2,"Configure");
      lcd_printlnf(true ,3," 192.168.4.1");
      lcd_printlnf(false,4,"Datalog");
      lcd_printlnf(false,5," 192.168.4.1/datalog");
		ESP_LOGI(TAG, "Wifi access point ESP32GpsVario starting...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP("ESP32GpsVario");
      IPAddress myIP = WiFi.softAPIP();
	   ESP_LOGI(TAG,"AP IP address: %s", myIP.toString().c_str());
	   xTaskCreatePinnedToCore(&server_task, "servertask", 16384, NULL, 20, NULL, 1);
      }
   else {
      lcd_clear();
      vario_taskConfig();   	
	   xTaskCreatePinnedToCore(&vario_task, "variotask", 2048, NULL, 20, NULL, 1);
	   xTaskCreatePinnedToCore(&gps_task, "gpstask", 2048, NULL, 20, NULL, 0);
      // display_task lower priority than gps_task
	   xTaskCreatePinnedToCore(&display_task, "displaytask", 2048, NULL, 10, NULL, 0); 
      }

	while(1) {
      btn_debounce();
      if (BtnRPressed || BtnLPressed || BtnMPressed || Btn0Pressed) {
         IsFlashDisplayRequired = true;
         BacklitCounter = opt.misc.backlitSecs*33;   
         LCD_BKLT_ON();
         }
      if (BacklitCounter) {
         BacklitCounter--;
         if (BacklitCounter <= 0) LCD_BKLT_OFF();
         }
      if (BtnRPressed) {
         BtnRPressed = 0;
         ESP_LOGI(TAG,"Btn R");
         IsSpeakerEnabled = !IsSpeakerEnabled; // toggle speaker mute
         }
      if (BtnLPressed) {
         BtnLPressed = 0;
         ESP_LOGI(TAG,"Btn L");
         IsGpsHeading = !IsGpsHeading; // toggle between gps course heading and compass heading
         }
      if (BtnMPressed) {
         BtnMPressed = 0;
         ESP_LOGI(TAG,"Btn M");
         if (opt.misc.logType == LOGTYPE_IBG) {
            if (!IsLogging) {
               delayMs(2000);
               IsLogging = true;
               ESP_LOGI(TAG,"Logging started");
               }
            else {
               IsLogging = false;
               ESP_LOGI(TAG,"Logging stopped");
               }
            }
         }
      if (Btn0Pressed) {
         Btn0Pressed = 0;
         ESP_LOGI(TAG,"Btn 0");
         opt.misc.logType = (opt.misc.logType+1)%3;
         }

      delayMs(30);
      }
	}

