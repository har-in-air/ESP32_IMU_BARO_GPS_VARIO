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
6. Variometer audio feedback is more pleasant thanks to the esp32 onboard DAC and external audio amplifier driving
a cellphone speaker with sine-wave tones.

## Specifications
1. MPU9250 accelerometer+gyroscope+magnetometer sampled at 500Hz.
2. MS5611 barometric pressure sensor, sampled at 50Hz
3. Ublox compatible M8N gps module configured for 10Hz data rate with UBX binary protocol @115200 baud
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

## Software Build notes
Built with esp-idf build commit ffd4187883d69c5c39f2a1961fda06b51ed998fd

Uses Arduino-ESP32 (v0.0.1) as a component, so that we can take advantage of Arduino-ESP32 code for the Web server, and spi/gpio interfaces. See https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md for instructions on how to add the arduino component to an esp-idf project. It will appear as an 'arduino' sub-directory in the project /components directory. I haven't added the files to this repository due to the size and number of files. After adding the arduino component navigate to the /components/arduino/libraries directory and delete the SPIFFs sub-directory. I'm using SPIFFs code from https://github.com/loboris/ESP32_spiffs_example, and the Arduino code clashes with this.

### 'make menuconfig' changes from default values

#### SPIFFS configuration
1. Base address 0x180000
2. 65536 (64Kbytes) partition size
3. 4096 logical block size
4. page size = 256

#### Custom partition table
partitions.csv

Run 'make flashfs' once to create and flash the spiffs partition image.

#### Arduino configuration
1. Autostart arduino setup and loop : disable
2. Disable mutex locks for HAL : enable

#### ESP32 configuration
1. 80MHz clock
2. Main task stack size increased to 16384 to accommodate ESP32Webserver

#### PHY configuration 
1. Wifi tx power reduced to 13dB from 20dB. This reduces the 
current spikes on wifi transmit bursts, so no need for a honking big capacitor on the 
esp32 vcc line. The lower transmission power is not a problem for our application - if you're configuring the gpsvario 
from your pc/smartphone, the two units are going to be no more than a few feet apart.

#### Freertos tick rate
Increased to 200Hz from 100Hz, allows minimum tick delay 5mS instead of 10mS, reduces overhead of regular task yield
during server data download etc.

## Hardware
I used a ublox compatible gps module from Banggood (see screenshot in /pics directory). Not a great choice, it was expensive, and the smaller patch antenna meant that it doesn't get a fix in my apartment, while cheaper modules do (with a larger patch antenna). Plus, it doesn't save configuration settings to flash or eeprom. So it needs to be configured on  initialization each time.

I've uploaded a screenshot of an alternative ublox compatible module from Aliexpress that seems to be a better option. Cheaper, larger patch antenna, and with flash configuration save. I don't have one myself, am assuming the advertising is correct :-D. Note that we're trying to use  the highest fix rate possible (for future integration into the imu-vario algorithm) and Ublox documentation indicates that this is possible only when you restrict the module to one GPS constellation (GPS), rather than GPS+GLONASS  or GPS+GLONASS+BEIDOU. So don't waste your time looking for cheap multi-constellation modules.

For an external audio amplifier, you could go with the XPT8871, available on ebay and aliexpress. I used the MAX4410 because I had a few samples, and an already assembled breakout board from a previous project.

## Credits
1. Spiffs code from https://github.com/loboris/ESP32_spiffs_example
2. Sine-wave generation with ESP32 DAC modified from https://github.com/krzychb/dac-cosine
3. Web server library from https://github.com/Pedroalbuquerque/ESP32WebServer
3. Web server top level page handling, css styling modified from  https://github.com/G6EJD/ESP32-ESP8266-File-Download-Upload-Delete-Stream-and-Directory
4. MPU9250 initialization sequence modified from https://github.com/bolderflight/MPU9250/blob/master/MPU9250.cpp

## Issues


