#ifndef CONFIG_H_
#define CONFIG_H_

#include "sdkconfig.h"
#include "driver/gpio.h"


// low speed for configuring the sensors, high speed for readout
#define VSPI_CLK_CONFIG_FREQHZ  500000
#define VSPI_CLK_HIGH_FREQHZ    10000000

// imu, baro, flash use VSPI IOMux compatible pins

#define pinVSCLK     (18)
#define pinVMOSI	   (23)
#define pinVMISO	   (19)

#define pinImuCS	   (17) 
#define pinBaroCS	   (16)
#define pinFlashCS   (5)

#define IMU_CS_HI() 	   {GPIO.out_w1ts = (1 << pinImuCS);}
#define IMU_CS_LO() 	   {GPIO.out_w1tc = (1 << pinImuCS);}
#define BARO_CS_HI() 	{GPIO.out_w1ts = (1 << pinBaroCS);}
#define BARO_CS_LO() 	{GPIO.out_w1tc = (1 << pinBaroCS);}
#define FLASH_CS_HI()   {GPIO.out_w1ts = (1 << pinFlashCS);}
#define FLASH_CS_LO()   {GPIO.out_w1tc = (1 << pinFlashCS);}

#define pinDRDYINT	(4)


#define pinGpsTXD  (21)
#define pinGpsRXD  (22)
#define pinGpsRTS  (-1)
#define pinGpsCTS  (-1)

#define GPS_UART_NUM          UART_NUM_1
//#define UART_RX_BUFFER_SIZE   512
#define UART_RX_BUFFER_SIZE   256

#define pinBtn0		(0)

#define pinBtnL   36
#define pinBtnM   34
#define pinBtnR   39

#define BTN0()	      ((GPIO.in >> pinBtn0) & 0x1)

#define BTNL()  ((GPIO.in1.val >> (pinBtnL - 32)) & 0x1)
#define BTNM()  ((GPIO.in1.val >> (pinBtnM - 32)) & 0x1)
#define BTNR()  ((GPIO.in1.val >> (pinBtnR - 32)) & 0x1)

// lcd uses HSPI IOMux compatible pins
#define pinHSCLK	   14
#define pinHMOSI	   13 
#define pinHMISO     (-1)

#define pinLcdCS	   12
#define pinLcdRST    27
#define pinLcdRS     26
#define pinLcdBklt   33

#define LCD_CS_HI() 	   {GPIO.out_w1ts = (1 << pinLcdCS);}
#define LCD_CS_LO() 	   {GPIO.out_w1tc = (1 << pinLcdCS);}
#define LCD_RST_HI() 	{GPIO.out_w1ts = (1 << pinLcdRST);}
#define LCD_RST_LO() 	{GPIO.out_w1tc = (1 << pinLcdRST);}
#define LCD_RS_HI() 	   {GPIO.out_w1ts = (1 << pinLcdRS);}
#define LCD_RS_LO() 	   {GPIO.out_w1tc = (1 << pinLcdRS);}
#define LCD_BKLT_OFF()  {GPIO.out1_w1ts.val = ((uint32_t)1 << (pinLcdBklt - 32));}
#define LCD_BKLT_ON()   {GPIO.out1_w1tc.val = ((uint32_t)1 << (pinLcdBklt - 32));}

#define HSPI_CLK_FREQHZ 4000000

#define pinLED		      2

#define LED_ON() 		   {GPIO.out_w1ts = (1 << pinLED);}
#define LED_OFF()		   {GPIO.out_w1tc = (1 << pinLED);}

#define pinAudioAmpEna       32
#define pinAudioDAC          25

#define AUDIO_AMP_ENABLE()   {GPIO.out1_w1ts.val = ((uint32_t)1 << (pinAudioAmpEna - 32));}
#define AUDIO_AMP_DISABLE()  {GPIO.out1_w1tc.val = ((uint32_t)1 << (pinAudioAmpEna - 32));}


#define FLASH_W25Q128  // 128Mbit 104MHz


////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

#define UTC_OFFSET_MINS_MIN        -720
#define UTC_OFFSET_MINS_DEFAULT     330
#define UTC_OFFSET_MINS_MAX         720

#define BACKLIT_SECS_MIN             5
#define BACKLIT_SECS_DEFAULT        30
#define BACKLIT_SECS_MAX            60

#define TRACK_INTERVAL_SECS_MIN        1
#define TRACK_INTERVAL_SECS_DEFAULT    3
#define TRACK_INTERVAL_SECS_MAX        60

// IIR filter coefficient for glide ratio calculation
// higher number means more damping, 0 => no filtering
#define GLIDE_RATIO_IIR_MIN            0
#define GLIDE_RATIO_IIR_DEFAULT        90
#define GLIDE_RATIO_IIR_MAX            99

// position DOP required before gpsvario registers start position
// for automatic track logging
#define GPS_STABLE_DOP_MIN             3
#define GPS_STABLE_DOP_DEFAULT         6
#define GPS_STABLE_DOP_MAX             15

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	   20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	   100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	   -20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	   20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-200
#define VARIO_SINK_THRESHOLD_CPS_MIN    	   -400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	   -100


// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more perceived frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT          400
#define VARIO_CROSSOVER_CPS_MIN              300
#define VARIO_CROSSOVER_CPS_MAX              800

// Kalman filter configuration
// actual variance value used is accel_variance*1000
#define KF_ACCEL_VARIANCE_DEFAULT            50
#define KF_ACCEL_VARIANCE_MIN                10
#define KF_ACCEL_VARIANCE_MAX                100

#define KF_ZMEAS_VARIANCE_DEFAULT            300
#define KF_ZMEAS_VARIANCE_MIN                100
#define KF_ZMEAS_VARIANCE_MAX                500


// If you find that gyro calibration fails when you leave
// the unit undisturbed, possibly your unit has an MPU9250 device
// with a high gyro bias on one or more axes. Try increasing this limit  
// until you find the calibration works consistently.

#define GYRO_OFFSET_LIMIT_1000DPS_DEFAULT   	150
#define GYRO_OFFSET_LIMIT_1000DPS_MIN       	25
#define GYRO_OFFSET_LIMIT_1000DPS_MAX		   200


// Track logging and track elapsed time display will start when the gps
// detects a position at least <threshold> m away from the start
// position
#define TRACK_START_THRESHOLD_M_MIN           0
#define TRACK_START_THRESHOLD_M_DEFAULT       20
#define TRACK_START_THRESHOLD_M_MAX           100

// Local magnetic declination in degrees
// For correction, subtract declination from compass reading
// West declination is negative
#define MAG_DECLINATION_DEG_MIN      -60
#define MAG_DECLINATION_DEG_DEFAULT  0
#define MAG_DECLINATION_DEG_MAX      60

#define SPEAKER_VOLUME_MIN       0
#define SPEAKER_VOLUME_DEFAULT   2
#define SPEAKER_VOLUME_MAX       3


#define LOGTYPE_NONE  0
#define LOGTYPE_GPS   1 // gps track log with intervals from 1-60s
#define LOGTYPE_IBG   2 // high speed imu+baro+gps data samples log

#define WAYPOINT_RADIUS_M_MIN         5
#define WAYPOINT_RADIUS_M_MAX      20000
#define WAYPOINT_RADIUS_M_DEFAULT   50


///////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	400
#define VARIO_SPKR_MAX_FREQHZ       3200

// Two octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// Uncomment this if you want the current beep/tone to be interrupted and
// a new tone generated when there is a 'significant' change in climb/sink rate.
// This will give you faster apparent vario response, but could also be 
// confusing/irritating if you are in choppy air
//#define VARIO_INTERRUPT_BEEPS

// this is the 'significant change' threshold that is used when 
// VARIO_INTERRUPT_BEEPS is enabled
#define VARIO_DISCRIMINATION_THRESHOLD_CPS    25

#define KF_ACCELBIAS_VARIANCE   1.0f


// print debug information to the serial port for different code modules

// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define MAIN_DEBUG
#define KF_DEBUG
#define BEEPER_DEBUG
#define CALIB_DEBUG
#define OPT_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
#define IMU_DEBUG
//#define CCT_DEBUG



#endif
