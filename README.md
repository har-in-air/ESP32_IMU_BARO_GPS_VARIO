# ESP32 GPS Variometer

## Features
1. Variometer zero-lag response with a Kalman filter fusing acceleration data from an IMU module and altitude data from a barometric pressure sensor.
2. High-speed data logging option for IMU (accelerometer, gyrosocope and magnetometer), barometer and gps 
readings. 500Hz for IMU data, 50Hz for barometric altitude data, and 10Hz for gps data. Data is logged to
a 128Mbit serial SPI flash. This is useful for offline analysis and development of data processing algorithms.
3. Normal GPS track logging option with variable track interval from 1 to 60 seconds. Data is logged to a 
128Mbit serial SPI flash.
4. Wifi access for downloading data or track logs and configuring user options. The unit acts as a Wifi
access point and web server. So you can access the datalogs/configuration in the field with a smartphone.
5. Load a route from one of up to 7 route files in FormatGEO .wpt format that were previously uploaded to the gpsvario.
5. 128x64 LCD display of GPS altitude, climb/sink rate, distance from start/to waypoint, ground speed,
glide ratio, course/compass heading, bearing to start/waypoint, GPS derived clock, elapsed-time, battery, speaker, and data logging status.
6. Variometer audio feedback is more pleasant thanks to the esp32 onboard DAC and external audio amplifier driving
a cellphone speaker with sine-wave tones.
7. Flight log summaries (date, time, start and end coordinates, duration, max altitude, max climb and sink rates) are stored as text files in the spiffs file system and can be downloaded using wifi.

## Hardware notes
1. MPU9250 accelerometer+gyroscope+magnetometer sampled at 500Hz.
2. MS5611 barometric pressure sensor, sampled at 50Hz
3. Ublox compatible M8N gps module configured for 10Hz data rate with UBX binary protocol @115200 baud.
I used a gps module from Banggood (see screenshot in /pics directory). Not a great choice, it was expensive, and the smaller patch antenna meant that it doesn't get a fix in my apartment, while cheaper modules do (with a larger patch antenna). And it doesn't save configuration settings to flash or eeprom, so it needs to be configured on initialization each time.

I've uploaded a screenshot of an alternative ublox compatible module from Aliexpress that seems to be a better option. Cheaper, larger patch antenna, and with flash configuration save. I don't have one myself, am assuming the advertising is correct :-D. Note that we're trying to use  the highest fix rate possible (for future integration into the imu-vario algorithm). Ublox documentation indicates that this is possible only when you restrict the module to one GPS constellation (GPS), rather than GPS+GLONASS  or GPS+GLONASS+BEIDOU. So don't waste your time looking for cheap multi-constellation modules.
4. ESP32 WROOM rev 1 module
5. 128x64 reflective LCD display with serial spi interface.
6. MAX4410 audio amplifier driving salvaged 8ohm cellphone speaker.
7. For the power supply, I use a single-cell 18650 
power bank. I added an extra connector wired directly to the battery terminals. This allows me to 
detach the power bank and use it for other purposes, e.g. recharging my phone. And I can put 
my hand-wired gpsvario in checked-in luggage (no battery, no problem), with the power bank in my carry-on 
luggage as per airline requirements.
8. Average current draw is ~160mA in gpsvario mode, ~300mA in wifi access point mode. Not
 optimized. I haven't bothered to add a software controlled power switch for the gps module,
for example. If you're content with driving a piezo speaker, can save some power 
by omitting the audio amplifier, and using square-wave piezo drive.

For an external audio amplifier, you could go with the XPT8871, available on ebay and aliexpress. I used the MAX4410 because I had a few samples, and an already assembled breakout board from a previous project.

## Software notes
Compiled on Ubuntu 16.04LTS amdx64 platform 
(esp-idf git log : commit 84788230392d0918d3add78d9ccf8c2bb7de3152, 2018 Mar 21)

Uses Arduino-ESP32 (v0.0.1) as a component, so that we can take advantage of Arduino-ESP32 code for the Web server, and spi/gpio interfaces. See https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md for instructions on how to add the arduino component to an esp-idf project. It will appear as an 'arduino' sub-directory in the project /components directory. I haven't added the files to this repository due to the size and number of files. After adding the arduino component navigate to the /components/arduino/libraries directory and delete the SPIFFs sub-directory. I'm using SPIFFs code from https://github.com/loboris/ESP32_spiffs_example, and the Arduino code clashes with this.

### 'make menuconfig' changes from default values

#### SPIFFS configuration
1. Base address : 0x180000
2. Partition size : 65536 (64Kbytes)
3. Logical block size : 4096 
4. Page size : 256

#### Custom partition table
partitions.csv

(Run 'make flashfs' once to create and flash the spiffs partition image)

#### Arduino configuration
1. Autostart arduino setup and loop : disable
2. Disable mutex locks for HAL : enable

#### ESP32 configuration
1. Clock frequency : 80MHz (reduces power consumption)
2. Main task stack size : 16384 (increased to accommodate ESP32Webserver)

#### PHY configuration 
1. Wifi tx power : 13dB 
This reduces the current spikes on wifi transmit bursts, so no need for a honking big capacitor on the 
esp32 vcc line. The lower transmission power is not a problem for our application - if you're configuring the gpsvario 
from your pc/smartphone, the two units are going to be no more than a few feet apart.

#### Freertos
1. Tickrate : 200Hz
Allows minimum tick delay 5mS instead of 10mS, reduces overhead of regular task yield
during server data download etc.

## Usage
1. There are 4 user-interface buttons labeled as btnL(eft), btnM(iddle) and btnR(ight), plus btn0. An additional hardware reset button is used along with btn0 for putting the gpsvario into code flashing mode. 
2. For downloading binary data logs, put the gpsvario into server mode, connect to the WiFi access point 'ESP32GpsVario' and access the url 'http://192.168.4.1/datalog' via a web browser. The binary datalog file can contain a mix of high-speed IBG (imu+baro+gps) data samples, and normal GPS track logs. There is some sample software in the /offline directory for splitting the binary datalog into separate IBG and GPS datalogs, and for converting GPS logs into .gpx text files that you can load in Google Earth or other GPS track visualization software.
3. For configuring the gpsvario, you can edit the user-configurable options onscreen. Or the hard way, put it into server mode, access the url 'http://192.168.4.1' and download the options.txt file from the gpsvario. Edit it as required, and upload the file back to the gpsvario (you can still tweak the parameters on-screen). To reset to 'factory defaults', just delete the options.txt file from the gpsvario using the webpage.
4. Use xcplanner (xcplanner.appspot.com) to generate a route with waypoints in FormatGEO format as a *.wpt text file. Note that xcplanner does not specify waypoint radii in the FormatGEO file. You can edit the file to add the waypoint radius (in meters) at the end of each waypoint entry line. If the waypoint radius is not specified, the gpsvario will apply a user-configurable standard waypoint radius. Upload the waypoint file to the gpsvario using the webpage upload file function. You can upload up to 7 route files and select one of them (or none) on-screen.

## Credits
1. Spiffs code - https://github.com/loboris/ESP32_spiffs_example
2. Sine-wave generation with ESP32 DAC -  https://github.com/krzychb/dac-cosine
3. Web server library - https://github.com/Pedroalbuquerque/ESP32WebServer
3. Web server top level page handling, css style modified from  https://github.com/G6EJD/ESP32-ESP8266-File-Download-Upload-Delete-Stream-and-Directory
4. MPU9250 initialization sequence modified from https://github.com/bolderflight/MPU9250/blob/master/MPU9250.cpp

## Issues
Preliminary, unstable, work in progress ... 

