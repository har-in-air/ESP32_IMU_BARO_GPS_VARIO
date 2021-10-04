#include "common.h"
#include <FS.h>
#include <LITTLEFS.h>
#include <WiFi.h>              
#include <BluetoothSerial.h>
#include "common.h"
#include "config.h"
#include "drv/cct.h"
#include "drv/spiflash.h"
#include "drv/audio.h"
#include "drv/vspi.h"
#include "drv/hspi.h"
#include "drv/btn.h"
#include "drv/wdog.h"
#include "sensor/mpu9250.h"
#include "sensor/gps.h"
#include "sensor/madc.h"
#include "sensor/ringbuf.h"
#include "sensor/imu.h"
#if USE_MS5611
#include "sensor/ms5611.h"
#elif USE_BMP388
#include "sensor/bmp388.h"
#endif
#include "sensor/kalmanfilter4.h"
#include "nv/flashlog.h"
#include "nv/calib.h"
#include "nv/options.h"
#include "bt/btmsg.h"
#include "wifi/async_server.h"
#include "ui/lcd7565.h"
#include "ui/beeper.h"
#include "ui/ui.h"
#include "ui/route.h"

static const char* TAG = "main";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

volatile int LedState = 0;
volatile float KFAltitudeCm, KFClimbrateCps,DisplayClimbrateCps;
volatile float YawDeg, PitchDeg, RollDeg;
volatile SemaphoreHandle_t DrdySemaphore;
volatile bool DrdyFlag = false;

int BacklitCounter;
bool IsGpsInitComplete = false;
bool IsServer = false; 
BluetoothSerial* pSerialBT = NULL;

static void pinConfig();
static void vario_taskConfig();
static void btserial_task(void *pvParameter);
static void ui_task(void *pvParameter);
static void gps_task(void *pvParameter);
static void vario_task(void *pvParameter);
static void main_task(void* pvParameter);
static void IRAM_ATTR drdyHandler(void);

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
#if USE_MS5611    
    pinMode(pinMS5611CS, OUTPUT);
    MS5611_CS_HI();
#elif USE_BMP388    
    pinMode(pinBMP388CS, OUTPUT);
    BMP388_CS_HI();
#endif
    FLASH_CS_HI();
    IMU_CS_HI();

    // using NS8002 amp module, external 100K pullup resistor to 5V. Pull down to enable.
    pinMode(pinAudioAmpEna, OUTPUT_OPEN_DRAIN); 
    AUDIO_AMP_DISABLE();

    pinMode(pinBtn0, INPUT_PULLUP); 
    pinMode(pinBtnL, INPUT); // external 10K pullup
    pinMode(pinBtnM, INPUT); // external 10K pullup
    pinMode(pinBtnR, INPUT); // external 10K pullup

    pinMode(pinLED, OUTPUT);
    LED_OFF();

    pinMode(pinDRDYINT, INPUT); // external 10K pullup
    }


static void btserial_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting btserial task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
	char szmsg[100];

    while (1) {
		if (opt.misc.btMsgType == BT_MSG_LK8EX1) {
			btmsg_genLK8EX1(szmsg);
			}
		else
		if (opt.misc.btMsgType == BT_MSG_XCTRC) {
			btmsg_genXCTRC(szmsg);
			}
        pSerialBT->print(szmsg);
		delayMs(1000/opt.misc.btMsgFreqHz);
		}
    vTaskDelete(NULL);
    }


static void ui_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting ui task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    int counter = 0;
    NAV_PVT navpvt;
    TRACK  track;
    IsGpsFixStable = false;
    IsLoggingIBG = false;
    IsGpsTrackActive = false;
    IsLcdBkltEnabled = false;
    IsSpeakerEnabled = true;
    EndGpsTrack = false;

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
        if (IsGpsTrackActive && EndGpsTrack) {
            IsGpsTrackActive = false;
            lcd_clear_frame();
            lcd_printlnf(false, 1, "%4d/%02d/%02d %02d:%02d", track.year, track.month, track.day, track.hour, track.minute);
            lcd_printlnf(false, 2, "Duration %02dhr %02dmin", track.elapsedHours, track.elapsedMinutes);
            lcd_printlnf(false, 3, "Alt St %4dm Mx %4dm", track.startAltm, track.maxAltm);
            lcd_printlnf(false, 4, "Max Climb +%.1fm/s", track.maxClimbrateCps/100.0f);
            lcd_printlnf(true, 5, "Max Sink  %.1fm/s", track.maxSinkrateCps/100.0f);
            ui_saveFlightLogSummary(&navpvt, &track);
            while(1) delayMs(100);
            }

		delayMs(5);
		}
    vTaskDelete(NULL);
	}	


static void gps_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting gps task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    if (!gps_config()) {
        ESP_LOGE(TAG, "error configuring gps");
		lcd_clear_frame();
		lcd_printlnf(true,3,"GPS init error");
        while (1) delayMs(100);
        }
    IsGpsInitComplete = true;
    while(1) {
        gps_stateMachine();
        delayMs(5);
        }
    vTaskDelete(NULL);
    }  


static void IRAM_ATTR drdyHandler(void) {
	DrdyFlag = true;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up vario_task immediately instead of on next FreeRTOS tick
		}
	//LED_TOGGLE();
	}


static void vario_taskConfig() {
    ESP_LOGI(TAG, "vario config");
    lcd_clear_frame();
    if (mpu9250_config() < 0) {
        ESP_LOGE(TAG, "error MPU9250 config");
		lcd_printlnf(true,3,"MPU9250 config failed");
        while (1) {delayMs(100);};
        }

    btn_clear();
    // if calib.txt not found, enforce accel and mag calibration and write new calib.txt
    bool isAccelMagCalibRequired = !IsCalibrated;
    int counter = 300;
    while ((!isAccelMagCalibRequired) && counter--) {
   	    lcd_printlnf(true,3,"Gyro calib in %ds",(counter+50)/100);
        if (BTN0() == LOW) {
            // manually force accel+mag calibration by pressing BTN0 during gyro calib countdown
            isAccelMagCalibRequired = true; 
            break;
            }
        delayMs(10);	
		}
    if (isAccelMagCalibRequired) {
        counter = 8;
        while (counter--) {
            lcd_printlnf(true,3,"Accel calib in %ds",counter+1);
            delayMs(1000);	
            }
        lcd_printlnf(true,3, "Calibrating Accel...");
        if (mpu9250_calibrateAccel() < 0) {
 	        lcd_printlnf(true,3,"Accel calib failed");
            while (1) {delayMs(100);};
            }
        delayMs(1000);
        counter = 5;
        while (counter--) {
            lcd_printlnf(true,3,"Mag calib in %ds",counter+1);
            delayMs(1000);	
            }
        lcd_printlnf(true,3,"Calibrating Mag...");
        if (mpu9250_calibrateMag()  < 0 ) {
 	        lcd_printlnf(true,3,"Mag calib failed");
            }
        }
    if (isAccelMagCalibRequired) {
        counter = 3;
        while (counter--) {
            lcd_printlnf(true,3,"Gyro calib in %ds",counter+1);
            }
        }
    lcd_printlnf(true,3,"Calibrating Gyro...");
    if (mpu9250_calibrateGyro() < 0) {
 	    lcd_printlnf(true,3,"Gyro calib fail");
        delayMs(1000);
        }
#if USE_MS5611
    if (ms5611_config() < 0) {
        ESP_LOGE(TAG, "error MS5611 config");
        lcd_printlnf(true,3, "MS5611 config fail");
        while (1) {delayMs(100);}
        }	
    ms5611_averagedSample(20);
    ESP_LOGD(TAG,"MS5611 Altitude %dm Temperature %dC", (int)(ZCmAvg_MS5611/100.0f), (int)CelsiusSample_MS5611);
    float zcm = ZCmAvg_MS5611;
    ms5611_initializeSampleStateMachine();
#elif USE_BMP388    
    if (bmp388_config() < 0) {
        ESP_LOGE(TAG, "error BMP388 config");
        lcd_printlnf(true,3, "BMP388 config fail");
        while (1) {delayMs(100);}
        }	
    {
    uint32_t marker =  cct_setMarker();
    bmp388_sample();
    uint32_t eus = cct_elapsedUs(marker);
    ESP_LOGD(TAG, "BMP388 sample and pa2z : %d us", eus);
    }
    
    bmp388_averaged_sample(20);
    ESP_LOGD(TAG,"BMP388 Altitude %dm Temperature %dC", (int)(ZCmAvg_BMP388/100.0f), (int)CelsiusSample_BMP388);
    float zcm = ZCmAvg_BMP388;
#endif

    // KF4D algorithm to fuse gravity-compensated acceleration and pressure altitude to estimate
    // altitud and climb/sink rate
    kalmanFilter4_configure((float)opt.kf.zMeasVariance, 1000.0f*(float)opt.kf.accelVariance, true, zcm, 0.0f, 0.0f);

    lcd_clear_frame();
    lcd_printlnf(true,3,"Baro Altitude %dm", (int)(zcm/100.0f));
    // switch to high clock frequency for sensor readout & flash writes
    vspi_setClockFreq(VSPI_CLK_HIGH_FREQHZ); 
    beeper_config();
    ringbuf_init(); 
    }


static void vario_task(void *pvParameter) {
    float gxdps, gydps, gzdps, axmG, aymG, azmG, mx, my, mz;
    float gxNEDdps, gyNEDdps, gzNEDdps, axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED;

    ESP_LOGI(TAG, "Starting vario task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
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
#if USE_MS5611
        if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
            baroCounter = 0;     // alternating between pressure and temperature samples
            int zMeasurementAvailable = ms5611_sampleStateMachine(); 
            // one altitude sample is calculated for a pair of pressure & temperature samples
			if ( zMeasurementAvailable ) { 
                // KF4 uses the acceleration data in the update phase
                float zAccelAverage = ringbuf_averageNewestSamples(10); 
                kalmanFilter4_predict(kfTimeDeltaUSecs/1000000.0f);
                kalmanFilter4_update(ZCmSample_MS5611, zAccelAverage, (float*)&KFAltitudeCm, (float*)&KFClimbrateCps);
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
				        FlashLogIBGRecord.baro.heightMSLcm = ZCmSample_MS5611;
					    xSemaphoreGive( FlashLogMutex );
					    }
				    }
                }    
            } 
#elif USE_BMP388
        if (baroCounter >= 10) { // 10*2mS = 20mS elapsed, this is the configured sampling period for BMP388
            baroCounter = 0;     
            bmp388_sample();
            // KF4 uses the acceleration data in the update phase
            float zAccelAverage = ringbuf_averageNewestSamples(10); 
            kalmanFilter4_predict(kfTimeDeltaUSecs/1000000.0f);
            kalmanFilter4_update(ZCmSample_BMP388, zAccelAverage, (float*)&KFAltitudeCm, (float*)&KFClimbrateCps);
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
	                FlashLogIBGRecord.baro.heightMSLcm = ZCmSample_BMP388;
					xSemaphoreGive( FlashLogMutex );
					}
				}
            }    
#endif
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
                if (IsLoggingIBG) {
                    // out of memory, indicate in UI that logging has stopped
   	                if (flashlog_writeIBGRecord(&FlashLogIBGRecord) < 0) {
                        IsLoggingIBG = false;
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
            //lcd_clear_frame();
            //lcd_printf(0,0,"a %d %d %d", (int)axmG, (int)aymG, (int)azmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxdps, (int)gydps, (int)gzdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mx, (int)my, (int)mz);
            //lcd_printf(0,0,"a %d %d %d", (int)axNEDmG, (int)ayNEDmG, (int)azNEDmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxNEDdps, (int)gyNEDdps, (int)gzNEDdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mxNED, (int)myNED, (int)mzNED);
            //lcd_send_frame();
#endif
            }
        }
    vTaskDelete(NULL);
    }


static void main_task(void* pvParameter) {
    pinConfig();
    // turn off radio to save power
    WiFi.mode(WIFI_OFF);
    btStop();
    ESP_LOGI(TAG, "Firmware compiled on %s at %s", __DATE__, __TIME__);
    ESP_LOGD(TAG, "Max task priority = %d", configMAX_PRIORITIES-1);
    ESP_LOGD(TAG, "Setup and loop running on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    
    ESP_LOGD(TAG, "Mounting LITTLEFS ...");
    // do NOT format, partition is built and flashed using PlatformIO Build FileSystem Image + Upload FileSystem Image    
    if (!LITTLEFS.begin(false)) { 
	ESP_LOGE(TAG, "Cannot mount LITTLEFS, Rebooting");
	delay(1000);
	ESP.restart();
	}    
    //littlefs_directory_listing();
    // read calibration parameters from calib.txt
    calib_init();
    // set default configuration parameters, then override them with configuration parameters read from options.txt
    // so you only have to specify the parameters you want to modify, in the file options.txt
    opt_init();
    // configure DAC sine-wave tone generation
    audio_config(pinAudioDAC);
 
    // HSPI bus used for 128x64 LCD display
    hspi_config(pinHSCLK,-1, pinHMOSI,-1,HSPI_CLK_FREQHZ);
    lcd_init(opt.misc.lcdContrast);
    LCD_BKLT_ON();
    BacklitCounter = opt.misc.backlitSecs*40;
    adc_init();
    uint32_t supplyVoltagemV = adc_supplyVoltageMV();
    ESP_LOGD(TAG, "Power supply = %.1fV", (float) supplyVoltagemV/1000.0f);
    lcd_printlnf(false,0,"%s %s", __DATE__, __TIME__);
    lcd_printlnf(true,1,"Power supply : %.1fV", (float) supplyVoltagemV/1000.0f);

    // VSPI bus used for MPU9250, MS5611 and 128Mbit spi flash
    // start with low clock frequency for sensor configuration
    vspi_config(pinVSCLK, pinVMOSI, pinVMISO, VSPI_CLK_CONFIG_FREQHZ);
    if (flashlog_init() < 0) {
	    ESP_LOGE(TAG, "Spi flash log error");
	    lcd_printlnf(true,3,"Flash Error");		
	    while (1) {delayMs(100);}
	    }

    lcd_printlnf(true,3,"Data log : %d %% used", DATALOG_PERCENT_USED()); 
    delayMs(2000);
    btn_clear();

    ESP_LOGI(TAG,"Press btn0 within 3 seconds to erase flash");
    LED_ON();
    bool isEraseRequired = false;
    int counter = 300;
    while (counter--) {
        lcd_printlnf(true,4,"btn0 to erase : %ds",(counter+50)/100);
        if (BTN0() == LOW) {
            ESP_LOGI(TAG,"btn0 PRESSED");
            isEraseRequired = true;
            break;
            }
        delayMs(10);	
        }
    if (isEraseRequired) {
        ESP_LOGI(TAG, "Erasing flash...");
	    spiflash_globalUnprotect();
        if (FlashLogFreeAddress == 0) {
            FlashLogFreeAddress = FLASH_SIZE_BYTES-1; // manually force erase of entire chip
            }
        uint32_t lastSectorAddress = FlashLogFreeAddress & 0xFFFFF000; // sector size = 4096
	    for (uint32_t sectorAddress = 0; sectorAddress <= lastSectorAddress; sectorAddress += 4096) {
		    spiflash_sectorErase(sectorAddress);
		    ESP_LOGD(TAG,"%4X",sectorAddress>>12);
		    lcd_printlnf(true,4, "Erasing %03X", sectorAddress>>12);
            feed_watchdog(); // flash erase takes time, feed watchdog to avoid reset
		    }
        spiflash_reset();
        FlashLogFreeAddress = 0;
        lcd_printlnf(false,3,"Data log : %d %% used", DATALOG_PERCENT_USED());
	    ESP_LOGI(TAG,"Erased");
        lcd_printlnf(true,4,"Erased");
        }
    delayMs(1000);
    IsServer = false;
    ESP_LOGI(TAG,"Press btn0 within 3 seconds to start WiFi AP and web server");
    counter = 300;
    while (counter--) {
        lcd_printlnf(true,4,"btn0 for wifi cfg: %ds",(counter+50)/100);
        if (BTN0() == LOW) {
            ESP_LOGI(TAG,"btn0 Pressed, starting WiFi AP and web server");
            IsServer = 1;
            break;
            }
        delayMs(10);
        }
    if (IsServer) { // Wifi Configuration mode
        lcd_clear_frame();
        lcd_printlnf(false,0,"WiFi Access Point :");
        lcd_printlnf(false,1," \"ESP32GpsVario\"");
        lcd_printlnf(false,3,"Web Page :");
        lcd_printlnf(false,4," http://esp32.local");
        lcd_printlnf(true,5," 192.168.4.1");
        ESP_LOGI(TAG, "Wifi access point ESP32GpsVario starting...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32GpsVario");
        IPAddress myIP = WiFi.softAPIP();
        ESP_LOGI(TAG,"WiFi Access Point IP address: %s", myIP.toString().c_str());
        LCD_BKLT_OFF();
        server_init();
        }
    else { // GPS Vario mode
        ui_screenInit();
        ui_displayOptions();
        btn_clear();
        while (ui_optionsEventHandler() == 0) {
            btn_debounce();
            delayMs(30);
            }
        if (rte_selectRoute()) {
            int32_t rteDistance = rte_totalDistance();
            lcd_clear_frame();
            lcd_printlnf(true,0,"Route %.2fkm", ((float)rteDistance)/1000.0f);
            delayMs(1000);
            IsRouteActive = true;
            }
        vario_taskConfig();   	
        // vario task on core 1 needs to complete all processing within 2mS IMU data sampling period,
        // given highest priority on core 1
	    xTaskCreatePinnedToCore(&vario_task, "variotask", 4096, NULL, configMAX_PRIORITIES-1, NULL, CORE_1);
        IsGpsInitComplete = false;
        // gps task on core 0 given max priority
	    xTaskCreatePinnedToCore(&gps_task, "gpstask", 2048, NULL, configMAX_PRIORITIES-1, NULL, CORE_0);
        while(!IsGpsInitComplete){
            delayMs(10);
            }
        // ui_task on core 0 given lower priority than gps_task
	    xTaskCreatePinnedToCore(&ui_task, "uitask", 4096, NULL, configMAX_PRIORITIES-3, NULL, CORE_0);
	    if (opt.misc.btMsgFreqHz != 0) {
            pSerialBT = new BluetoothSerial;
            if (pSerialBT) {
                pSerialBT->begin("ESP32GpsVario");
    		    IsBluetoothEnabled = true;
                // bluetooth serial task on core 0 given higher priority than ui task, less than gps task
    		    xTaskCreatePinnedToCore(&btserial_task, "btserialtask", 3072, NULL, configMAX_PRIORITIES-2, NULL, CORE_0);
                }
            else {
                ESP_LOGE(TAG, "Failure creating BluetoothSerial");
                }
	    	}
      }
    while (1) {
        loop();
        }
    }


// loop runs on core 1 with default priority = 1
void loop() {
    if (IsServer) {
        ESP_LOGV(TAG, "Loop priority = %d", uxTaskPriorityGet(NULL));
        delayMs(500); // delay() is required to yield to other tasks
        return;
        }    
    btn_debounce();
    if (BtnRPressed || BtnLPressed || BtnMPressed || Btn0Pressed) {
        // invert display for one frame to acknowledge button press
        IsFlashDisplayRequired = true; 
        // turn on backlight for user configurable time if a button is pressed
        BacklitCounter = opt.misc.backlitSecs*40;   // loop delay is ~25mS, 40*25 = 1second
        LCD_BKLT_ON();
        }
    if (BacklitCounter) {
        BacklitCounter--;
        if (BacklitCounter <= 0) LCD_BKLT_OFF();
        }
    if (BtnRPressed) {
        btn_clear();
        ESP_LOGV(TAG,"Btn R");
        IsSpeakerEnabled = !IsSpeakerEnabled; // mute/enable speaker
        }
    if (BtnLPressed) {
        btn_clear();
        ESP_LOGV(TAG,"Btn L");
        IsGpsHeading = !IsGpsHeading; // toggle between gps course heading and magnetic compass heading
        }
    if (BtnMPressed) {
        btn_clear();
        ESP_LOGV(TAG,"Btn M"); // start/stop high-speed IBG data logging. Does nothing for GPS track logging.
        if (opt.misc.logType == LOGTYPE_IBG) {
            if (!IsLoggingIBG) {
                IsLoggingIBG = true;
                ESP_LOGV(TAG,"IBG Logging started");
                }
            else {
                IsLoggingIBG = false;
                ESP_LOGV(TAG,"IBG Logging stopped");
                }
            }
        }
    if (Btn0Pressed) {
        btn_clear();
        ESP_LOGV(TAG,"Btn 0");
        if (IsGpsTrackActive)  {
            EndGpsTrack = true;
            IsSpeakerEnabled = false;
            }
        }
    delayMs(25);
    }

// workaround for not having access to 'make menuconfig' to configure the stack size for the
// setup and loop task is to create a new main task with desired stack size, and then delete setup 
// task. 
// Core 0 : ui, GPS, Bluetooth tasks
// Core 1 : setup() + loop(), wifi config / vario task
void setup() {
    xTaskCreatePinnedToCore(&main_task, "main_task", 16384, NULL, configMAX_PRIORITIES-4, NULL, CORE_1);
    vTaskDelete(NULL);
    }
