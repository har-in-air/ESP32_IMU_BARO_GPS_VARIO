# ESP32 GPS Variometer

1. Variometer zero-lag response with a Kalman filter fusing acceleration data from an IMU module and altitude data from a barometric pressure sensor.
2. High-speed data logging option for IMU (accelerometer, gyrosocope and magnetometer), barometer and gps 
readings. 500Hz for IMU data, 50Hz for barometric altitude data, and 10Hz for gps data. Data is logged to
a 128Mbit serial SPI flash. This is useful for offline analysis and development of data processing algorithms.
3. Normal GPS track logging option with variable track interval from 1 to 60 seconds. Data is logged to a 
128Mbit serial SPI flash.
4. Wifi access for downloading data or track logs and configuring user options. The unit acts as a Wifi
access point and web server. So you can access the datalogs/configuration in the field with a smartphone.
5. 128x64 LCD display of GPS altitude, climb/sink rate, distance from start/to waypoint, ground speed,
glide ratio, course/compass heading, bearing to start/waypoint, GPS derived clock, elapsed-time, battery, speaker, and data logging status.
6. Variometer audio feedback is more pleasing thanks to the esp32 onboard DAC and audio amplifier driving
a cellphone speaker with sine-wave tones.

## Technical specifications
1. MPU9250 accelerometer+gyroscope+magnetometer sampled at 500Hz.
2. MS5611 barometric pressure sensor, sampled at 50Hz
3. Ublox M8N gps module configured for 10Hz data rate with UBX binary protocol @115200 baud
4. ESP32 WROOM rev 1 module
5. 128x64 reflective LCD display with serial spi interface.
6. MAX4410 audio amplifier driving salvaged 8ohm cellphone speaker.
7. For the power supply, I use a store-bought single-cell 18650 
power bank with an extra connector connecting directly to the battery terminals. This allows me to 
detach the power bank and use it for other purposes, e.g. recharging my phone etc. And I can put 
my hand-wired gpsvario in checked-in luggage (no battery), with the power bank in my carry-on 
luggage as per airline requirements.
8. Average current draw is ~160mA in gpsvario mode, ~300mA in wifi access point mode. Not
 optimized. I haven't bothered to add a software controlled power switch for the gps module,
for example. Got lazy because of my use of a 3000mAH 18650 battery :-).
If you're content with driving a piezo speaker, can save some power 
by omitting the audio amplifier, and using square-wave piezo drive.
9. Software uses esp-idf build environment with Arduino as a component, so that we can take advantage of Arduino-ESP32 code for the spi and gpio interfaces and the web server.  See 
https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md for instructions on how to add the arduino component to an esp-idf project - it will appear as an 'arduino' sub-directory in the project /components directory. I haven't added the files to this repository due to the size and number of files.

## Build notes
Uses esp-idf build commit ffd4187883d69c5c39f2a1961fda06b51ed998fd

Uses arduino component version=0.0.1

make menuconfig changes from default values

### Spiffs configuration
Base address 0x180000, 65536 (64Kbytes) partition size, 4096 logical block
size, page size = 256

### Custom partition table
partitions.csv

### Arduino configuration
Autostart arduino setup and loop : disable
Disable mutex locks for HAL : enable

### ESP32 configuration
80MHz clock, main task stack size increased to 16384 to accommodate ESP32Webserver

### PHY configuration 
wifi tx power reduced to 13dB from 20dB. This reduces the 
current spikes on wifi transmit bursts, so no need for a honking big capacitor on the 
esp32 vcc line. This is not a problem for our application - if you're configuring the gpsvario 
with a pc or smartphone, the two are going to be no more than a few feet apart.

### Freertos tick rate
Increased to 200Hz from 100Hz
allows minimum tick delay 5mS instead of 10mS, reduces overhead of regular task yield
during server data download etc.

## Issues


