#include <Arduino.h>
#include <WiFi.h>              
#include <ESP32WebServer.h>
#include "server.h"

extern "C" {
#include "common.h"
#include "config.h"
#include "spiffs_vfs.h"
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
#include "route.h"
#include "options.h"
#include "btspp.h"
#include "btmsg.h"
}

#define TAG "main"

ESP32WebServer *pServer = NULL;

volatile int LedState = 0;
volatile float KFAltitudeCm, KFClimbrateCps,DisplayClimbrateCps;
volatile float YawDeg, PitchDeg, RollDeg;

volatile SemaphoreHandle_t DrdySemaphore;
volatile bool DrdyFlag = false;

bool IsServer = false; 

void pinConfig() {	
    pinMode(pinLcdBklt, OUTPUT);
    LCD_BKLT_OFF();
    pinMode(pinLcdRST, OUTPUT);
    pinMode(pinLcdCS, OUTPUT);
    LCD_RST_HI();
    LCD_CS_HI();
    pinMode(pinLcdA0, OUTPUT);

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



static void btserial_task(void *pvParameter) {
    ESP_LOGD(TAG, "Starting bluetooth serial output : device ESP32GpsVario");
    while (1) {
		char szmsg[100];
		if (opt.misc.btMsgType == BT_MSG_LK8EX1) {
			btmsg_genLK8EX1(szmsg);
			}
		else
		if (opt.misc.btMsgType == BT_MSG_XCTRC) {
			btmsg_genXCTRC(szmsg);
			}
		btspp_print(szmsg,strlen(szmsg));
		delayMs(1000/opt.misc.btMsgFreqHz);
		}
    vTaskDelete(NULL);
    }



static void server_task(void *pvParameter){
	pServer = new ESP32WebServer(80);
	if (pServer == NULL) {
		ESP_LOGE(TAG, "error malloc webserver");
		lcd_clear();
		lcd_printlnf(true,0,"webserver error");
		while(1) delayMs(100);
		}
    pServer->on("/",         server_homePage);
    pServer->on("/dir",      server_listDir);
    pServer->on("/upload",   server_fileUpload);
    pServer->on("/fupload",  HTTP_POST,[](){ pServer->send(200);}, server_handleFileUpload);
    pServer->on("/download", server_fileDownload);
    pServer->on("/delete",   server_fileDelete);
    pServer->on("/datalog",  server_downloadDataLog);
    pServer->begin();
    while(1) {
        pServer->handleClient();
        delayMs(5);
        }
    delete pServer;
    vTaskDelete(NULL);
    }


static void ui_task(void *pvParameter) {
    int counter = 0;
    NAV_PVT navpvt;
    TRACK  track;
    IsGpsFixStable = false;
    IsLogging = false;
    IsTrackActive = false;
    IsLcdBkltEnabled = false;
    IsSpeakerEnabled = true;
    EndTrack = false;

    while(1) {
		if (IsGpsNavUpdated) {
			IsGpsNavUpdated = false;
            counter++;
            if (counter >= 5) {
                counter = 0;
                // GPS update interval = 0.1s => display update interval =  0.5s
                memcpy(&navpvt, (void*)&NavPvt, sizeof(NAV_PVT)); 
                ui_updateFlightDisplay(&navpvt,&track);
                }
			}
        if (IsTrackActive && EndTrack) {
            IsTrackActive = false;
            lcd_clear();
            lcd_printlnf(false, 0, "%4d/%02d/%02d %02d:%02d", track.year, track.month, track.day, track.hour, track.minute);
            lcd_printlnf(false, 1, "Duration %02d:%02d", track.elapsedHours, track.elapsedMinutes);
            lcd_printlnf(false, 2, "Alt s %4dm max %4dm", track.startAltm, track.maxAltm);
            lcd_printlnf(false, 3, "Max Climb +%.1fm/s", track.maxClimbrateCps/100.0f);
            lcd_printlnf(true, 4, "Max Sink %.1fm/s", track.maxSinkrateCps/100.0f);
            ui_saveFlightLogSummary(&navpvt, &track);
            while(1) delayMs(100);
            }

		delayMs(5);
		}
    vTaskDelete(NULL);
	}	


static void gps_task(void *pvParameter) {
	ESP_LOGI(TAG, "gps task started");
    if (!gps_config()) {
        ESP_LOGE(TAG, "error configuring gps");
		lcd_clear();
		lcd_printlnf(true,0,"gps error");
        delayMs(3000);
        lcd_clear();
        }
    while(1) {
        gps_stateMachine();
        delayMs(5);
        }
    vTaskDelete(NULL);
    }  


void IRAM_ATTR drdyHandler(void) {
	DrdyFlag = true;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up vario_task immediately instead of on next FreeRTOS tick
		}
	//LED_TOGGLE();
	}


static void vario_taskConfig() {
    lcd_clear();
    //lcd_printlnf(true,0,"MPU9250 config");
    if (mpu9250_config() < 0) {
        ESP_LOGE(TAG, "error MPU9250 config");
		lcd_printlnf(true,0,"MPU9250 config failed");
        while (1) {delayMs(100);};
        }

    btn_clear();
    // if calib.txt not found, enforce accel and mag calibration and write new calib.txt
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
        lcd_printlnf(true,0, "Calibrating Accel...");
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
 	//lcd_printlnf(true,0,"Baro config");
	if (ms5611_config() < 0) {
		ESP_LOGE(TAG, "error MS5611 config");
		lcd_printlnf(true,0, "Baro config failed");
		while (1) {delayMs(100);}
		}	

    ms5611_averagedSample(4);
    ESP_LOGD(TAG,"Baro Altitude %dm Temperature %dC", (int)(ZCmAvg/100.0f), (int)CelsiusSample);
    //lcd_printlnf(true,0,"Baro %dm %dC",(int)(ZCmAvg/100.0f), (int)CelsiusSample);
    kalmanFilter3_configure((float)opt.kf.zMeasVariance, 1000.0f*(float)opt.kf.accelVariance, KF_ACCELBIAS_VARIANCE, ZCmAvg, 0.0f, 0.0f);
    //delayMs(2000);
    lcd_printlnf(true,0,"GPS Vario start...");
	ms5611_initializeSampleStateMachine();
	vspi_setClockFreq(VSPI_CLK_HIGH_FREQHZ); // high clock frequency ok for sensor readout & flash writes
    beeper_config();
    ringbuf_init(); 
    }



static void vario_task(void *pvParameter) {
    float gxdps, gydps, gzdps, axmG, aymG, azmG, mx, my, mz;
    float gxNEDdps, gyNEDdps, gzNEDdps, axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED;

    ESP_LOGI(TAG, "vario task started");
    uint32_t clockPrevious, clockNow; // time markers for imu, baro and kalman filter
    float 	imuTimeDeltaUSecs; // time between imu samples, in microseconds
    float 	kfTimeDeltaUSecs = 0.0f; // time between kalman filter updates, in microseconds

    clockNow = clockPrevious = XTHAL_GET_CCOUNT();
    int drdyCounter = 0;   
    int baroCounter = 0;
    DrdySemaphore = xSemaphoreCreateBinary();
    attachInterrupt(pinDRDYINT, drdyHandler, RISING);

    while (1) {
        xSemaphoreTake(DrdySemaphore, portMAX_DELAY); // wait for data ready interrupt from MPU9250 (500Hz)
        clockNow = XTHAL_GET_CCOUNT();
#ifdef IMU_DEBUG
        uint32_t marker =  cct_setMarker();
        LED_ON();
#endif
        imuTimeDeltaUSecs = cct_intervalUs(clockPrevious, clockNow); // time in us since last sample
        clockPrevious = clockNow;
        drdyCounter++;
        baroCounter++;
        mpu9250_getGyroAccelMagData( &gxdps, &gydps, &gzdps, &axmG, &aymG, &azmG, &mx, &my, &mz);
        // translate from sensor axes to AHRS NED (north-east-down) right handed coordinate frame      
        axNEDmG = -aymG;
        ayNEDmG = -axmG;
        azNEDmG = azmG;
        gxNEDdps = gydps;
        gyNEDdps = gxdps;
        gzNEDdps = -gzdps;
        mxNED =  mx;
        myNED =  my;
        mzNED =  mz;
        // Use accelerometer data for determining the orientation quaternion only when accel 
        // vector magnitude is in [0.75g, 1.25g] window.
        float asqd = axNEDmG*axNEDmG + ayNEDmG*ayNEDmG + azNEDmG*azNEDmG;
        int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
        int useMag = true;
        imu_mahonyAHRSupdate9DOF(useAccel, useMag,((float)imuTimeDeltaUSecs)/1000000.0f, DEG2RAD(gxNEDdps), DEG2RAD(gyNEDdps), DEG2RAD(gzNEDdps), axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED);
        imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&YawDeg, (float*)&PitchDeg, (float*)&RollDeg);
        float gravityCompensatedAccel = imu_gravityCompensatedAccel(axNEDmG, ayNEDmG, azNEDmG, q0, q1, q2, q3);
        ringbuf_addSample(gravityCompensatedAccel); 
        kfTimeDeltaUSecs += imuTimeDeltaUSecs;

        if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
            baroCounter = 0;     // alternating between pressure and temperature samples
            int zMeasurementAvailable = ms5611_sampleStateMachine(); 
            // one altitude sample is calculated for a pair of pressure & temperature samples
			if ( zMeasurementAvailable ) { 
                // need average earth-z acceleration over the 20mS interval between z samples
                // z sample is from when pressure conversion was triggered, not read (i.e. 10mS ago). 
                // So we need to average the acceleration samples from the 20mS interval before that
                float zAccelAverage = ringbuf_averageOldestSamples(10); 
                kalmanFilter3_update(ZCmSample, zAccelAverage, ((float)kfTimeDeltaUSecs)/1000000.0f, (float*)&KFAltitudeCm, (float*)&KFClimbrateCps);
                kfTimeDeltaUSecs = 0.0f;
                // LCD display shows damped climbrate
                DisplayClimbrateCps = (DisplayClimbrateCps*(float)opt.vario.varioDisplayIIR + KFClimbrateCps*(100.0f - (float)opt.vario.varioDisplayIIR))/100.0f; 
                int32_t audioCps = INTEGER_ROUNDUP(KFClimbrateCps);
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
                FlashLogIBGRecord.hdr.magic = FLASHLOG_IBG_MAGIC;
			  	FlashLogIBGRecord.imu.gxNEDdps = gxNEDdps;
			    FlashLogIBGRecord.imu.gyNEDdps = gyNEDdps;
			    FlashLogIBGRecord.imu.gzNEDdps = gzNEDdps;
			    FlashLogIBGRecord.imu.axNEDmG = axNEDmG;
			    FlashLogIBGRecord.imu.ayNEDmG = ayNEDmG;
			    FlashLogIBGRecord.imu.azNEDmG = azNEDmG;
			    FlashLogIBGRecord.imu.mxNED = mxNED;
			    FlashLogIBGRecord.imu.myNED = myNED;
			    FlashLogIBGRecord.imu.mzNED = mzNED;
                // worst case imu+baro+gps record = 80bytes,
                //  ~130uS intra-page, ~210uS across page boundary
                if (IsLogging) {
                    // out of memory, indicate in UI that logging has stopped
   	                if (flashlog_writeIBGRecord(&FlashLogIBGRecord) < 0) {
                        IsLogging = false;
                        } 
                    memset(&FlashLogIBGRecord, 0, sizeof(FLASHLOG_IBG_RECORD));
                    }
				xSemaphoreGive( FlashLogMutex );
				}			
	        }  
#ifdef IMU_DEBUG
        uint32_t eus = cct_elapsedUs(marker);
		LED_OFF(); // scope the led on-time to ensure worst case < 2mS
#endif
        if (drdyCounter >= 500) {
            drdyCounter = 0;
#ifdef IMU_DEBUG
            //ESP_LOGD(TAG,"%dus",eus); // need to ensure time elapsed is less than 2mS worst case
			//ESP_LOGD(TAG,"\r\nY = %d P = %d R = %d", (int)YawDeg, (int)PitchDeg, (int)RollDeg);
			//ESP_LOGD(TAG,"ba = %d ka = %d v = %d",(int)ZCmSample, (int)KFAltitudeCm, (int)KFClimbrateCps);

            // ESP_LOGD(TAG,"ax %.1f ay %.1f az %.1f", axmG, aymG, azmG);
            // ESP_LOGD(TAG,"gx %.1f gy %.1f gz %.1f", gxdps, gydps, gzdps);
            // ESP_LOGD(TAG,"mx %.1f my %.1f mz %.1f", mx, my, mz);
            // ESP_LOGD(TAG,"ax %.1f ay %.1f az %.1f", axNEDmG, ayNEDmG, azNEDmG);
            // ESP_LOGD(TAG,"gx %.1f gy %.1f gz %.1f", gxNEDdps, gyNEDdps, gzNEDdps);
            // ESP_LOGD(TAG,"mx %.1f my %.1f mz %.1f", mxNED, myNED, mzNED);

            // ESP_LOGD(TAG,"baro alt %dcm", (int)ZCmSample);
            //lcd_clear();
            //lcd_printf(0,0,"a %d %d %d", (int)axmG, (int)aymG, (int)azmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxdps, (int)gydps, (int)gzdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mx, (int)my, (int)mz);
            //lcd_printf(0,0,"a %d %d %d", (int)axNEDmG, (int)ayNEDmG, (int)azNEDmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxNEDdps, (int)gyNEDdps, (int)gzNEDdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mxNED, (int)myNED, (int)mzNED);
#endif
            }
        }
    vTaskDelete(NULL);
    }


extern "C" void app_main() {
    ESP_LOGI(TAG, "esp32gpsvario compiled on %s at %s", __DATE__, __TIME__);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        }
    ESP_ERROR_CHECK( ret );
    pinConfig();

    // initialize SPIFFS file system 
    vfs_spiffs_register();
    // read calibration parameters from calib.txt
    calib_init();
    // set default configuration parameters, then override them with configuration parameters read from options.txt
    // so you only have to specify the parameters you want to modify, in the file options.txt
    opt_init();

    // configure DAC sine-wave tone generation
    audio_config(pinAudioDAC);
    // turn off wifi radio to save power
    WiFi.mode(WIFI_OFF);

    // HSPI bus used for 128x64 LCD display
    hspi_config(pinHSCLK,pinHMISO, pinHMOSI,-1,HSPI_CLK_FREQHZ);
    lcd_init(opt.misc.lcdContrast);
    LCD_BKLT_ON();
    int backlitCounter = opt.misc.backlitSecs*33;
    adc_init();
    uint32_t supplyVoltagemV = adc_supplyVoltageMV();
    ESP_LOGI(TAG, "Supply voltage = %d.%03dV", supplyVoltagemV/1000, supplyVoltagemV%1000);
    lcd_printlnf(false,0,"%s %s", __DATE__, __TIME__);
    lcd_printlnf(true,1,"Power %d.%03dV", supplyVoltagemV/1000, supplyVoltagemV%1000);

    // VSPI bus used for MPU9250, MS5611 and 128Mbit spi flash
    // start with low clock frequency for sensor configuration
    vspi_config(pinVSCLK, pinVMOSI, pinVMISO,VSPI_CLK_CONFIG_FREQHZ);
    if (flashlog_init() < 0) {
	    ESP_LOGE(TAG, "Spi flash log error");
	    lcd_printlnf(true,2,"Flash Error");		
	    while (1) {delayMs(100);}
	    }

    lcd_printlnf(true,2,"SPI flash %.2f%% full", 100.0f*((float)FlashLogFreeAddress)/(float)FLASH_SIZE_BYTES );
    delayMs(2000);
    btn_clear();

    ESP_LOGI(TAG,"Press BTN0 within 3 seconds to erase flash");
    LED_ON();
    bool isEraseRequired = false;
    int counter = 300;
    while (counter--) {
        lcd_printlnf(true,3,"BTN0 erase %ds",(counter+50)/100);
        if (!BTN0()) {
            ESP_LOGI(TAG,"BTN0 PRESSED");
            isEraseRequired = true;
            break;
            }
        delayMs(10);	
        }
    if (isEraseRequired) {
        ESP_LOGI(TAG, "Erasing flash...");
        lcd_printlnf(true,3, FlashLogFreeAddress ? "Erasing logs" : "Erasing flash");
        flashlog_erase(FlashLogFreeAddress);

        ESP_LOGI(TAG, "Done");
        lcd_printlnf(false,2,"SPI flash %.2f%% full", 100.0f*((float)FlashLogFreeAddress)/(float)FLASH_SIZE_BYTES );
        lcd_printlnf(true,3,"Erased");
        }
    delayMs(1000);
    IsServer = false;
    ESP_LOGI(TAG,"Press BTN0 within 3 seconds to start http server");
    counter = 300;
    while (counter--) {
        lcd_printlnf(true,3,"BTN0 server %ds",(counter+50)/100);
        if (!BTN0()) {
            ESP_LOGI(TAG,"BTN0");
            IsServer = 1;
            break;
            }
        delayMs(10);
        }

    if (IsServer) {
        initArduino();
        lcd_clear();
        lcd_printlnf(false,0,"HTTP Server Mode");
        lcd_printlnf(false,1,"AP \"ESP32GpsVario\"");
        lcd_printlnf(false,2,"Configure");
        lcd_printlnf(false,3," 192.168.4.1/config");
        lcd_printlnf(false,4,"Datalog");
        lcd_printlnf(true, 5," 192.168.4.1/datalog");
        ESP_LOGI(TAG, "Wifi access point ESP32GpsVario starting...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32GpsVario");
        IPAddress myIP = WiFi.softAPIP();
        ESP_LOGI(TAG,"AP IP address: %s", myIP.toString().c_str());
        xTaskCreatePinnedToCore(&server_task, "servertask", 16384, NULL, 20, NULL, 1);
        }
    else {
        ui_screenInit();
        ui_displayOptions();
        btn_clear();
        while (ui_optionsEventHandler() == 0) {
            btn_debounce();
            delayMs(30);
            }

        if (rte_selectRoute()) {
            int32_t rteDistance = rte_totalDistance();
            lcd_clear();
            lcd_printlnf(true,0,"Route %.2fkm", ((float)rteDistance)/1000.0f);
            delayMs(2000);
            IsRouteActive = true;
            }
        vario_taskConfig();   	
	    xTaskCreatePinnedToCore(&vario_task, "variotask", 2048, NULL, 20, NULL, 1);
	    xTaskCreatePinnedToCore(&gps_task, "gpstask", 2048, NULL, 20, NULL, 0);
        // ui_task lower priority than gps_task
	    xTaskCreatePinnedToCore(&ui_task, "uitask", 2048, NULL, 10, NULL, 0);
	    if (opt.misc.btMsgFreqHz != 0) {
	    	if (btspp_init("Esp32GpsVario")) { //Bluetooth device name
	    		xTaskCreatePinnedToCore(&btserial_task, "btserialtask", 2048, NULL, 15, NULL, 0);
	    		}
	    	}
        }


    while(1) {
        btn_debounce();
        if (BtnRPressed || BtnLPressed || BtnMPressed || Btn0Pressed) {
            IsFlashDisplayRequired = true; // invert display for one frame to acknowledge button press
            backlitCounter = opt.misc.backlitSecs*33;   // turn on backlight for user configurable time if a button is pressed
            LCD_BKLT_ON();
            }
        if (backlitCounter) {
            backlitCounter--;
            if (backlitCounter <= 0) LCD_BKLT_OFF();
            }
        if (BtnRPressed) {
            btn_clear();
            ESP_LOGD(TAG,"Btn R");
            IsSpeakerEnabled = !IsSpeakerEnabled; // mute/enable speaker
            }
        if (BtnLPressed) {
            btn_clear();
            ESP_LOGD(TAG,"Btn L");
            IsGpsHeading = !IsGpsHeading; // toggle between gps course heading and magnetic compass heading
            }
        if (BtnMPressed) {
            btn_clear();
            ESP_LOGD(TAG,"Btn M"); // start/stop high-speed IBG data logging
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
            btn_clear();
            ESP_LOGD(TAG,"Btn 0");
            if (IsTrackActive)  {
                EndTrack = true;
                IsSpeakerEnabled = false;
                }
            }
    
        delayMs(30);
        }
	}

