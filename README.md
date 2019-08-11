# ESP32 GPS Variometer

* Variometer zero-lag response with a Kalman filter fusing acceleration data from an IMU module and altitude data from a barometric pressure sensor.
* High-speed data logging option for IMU (accelerometer, gyrosocope and magnetometer), barometer and gps 
readings. 500Hz for IMU data, 50Hz for barometric altitude data, and 10Hz for gps data. Data is logged to
a 128Mbit serial SPI flash. This is useful for offline analysis and development of data processing algorithms.
* Normal GPS track logging option with variable track interval from 1 to 60 seconds. Data is logged to a 
128Mbit serial SPI flash.
* Wifi access for downloading data or track logs and configuring user options. The unit acts as a Wifi
access point and web server. So you can access datalogs/configuration with a smartphone or laptop.
* Navigate a route with waypoints from one of up to 7 route files in FormatGEO .wpt format that were previously uploaded to the gpsvario.
* 128x64 LCD display of GPS altitude, climb/sink rate, distance from start/to waypoint, ground speed,
glide ratio, course/compass heading, bearing to start/waypoint, GPS derived clock, elapsed-time, battery, speaker, and data logging status.
* Variometer audio feedback uses the esp32 onboard DAC and external audio amplifier driving
an 8ohm cellphone speaker with sine-wave tones.
* Flight summaries (date, start time, start and end coordinates, duration, max altitude, max climb and sink rates) are stored as single line entries in the file "flightlog.txt" in the spiffs file system. This text file can be downloaded using wifi and opened in a spreadsheet (open as CSV file) for analysis.

## Technical specifications
* MPU9250 accelerometer+gyroscope+magnetometer sampled at 500Hz.
* MS5611 barometric pressure sensor, sampled at 50Hz
* Ublox M8N gps module configured for 10Hz data rate with UBX binary protocol @115200 baud.
I used a compact gps module from Banggood (see screenshot in /docs directory). Not a great choice, it was expensive, and 
it doesn't get a fix in my apartment, while cheaper modules with a larger patch antenna do get a fix. 
And it doesn't save configuration settings to flash or eeprom, so it needs to be configured on initialization each time.

I've uploaded a screenshot of an alternative ublox compatible module from Aliexpress that seems to be a better option. 
Cheaper, larger patch antenna, and with flash configuration save. I don't have one myself, I'm assuming the advertising is correct :-D. 
Note that we're trying to use  the highest fix rate possible (for future integration into the imu-vario algorithm). 
Ublox documentation indicates that this is possible only when you restrict the module to one GPS constellation (GPS), rather than GPS+GLONASS  or GPS+GLONASS+BEIDOU. So don't waste your time looking for expensive multi-constellation modules.
* ESP32 WROOM rev 1 module. Any off-the-shelf breakout board with an onboard USB-UART interface (CH340, CP2102 etc).
* 128x64 reflective LCD display (ST7565 controller) with serial spi interface.
* MAX4410 audio amplifier driving 8ohm cellphone speaker.
* For the power supply, I use a generic USB 5V output power bank. This allows me to 
detach the power bank and use it for other purposes, e.g. recharging my phone. And I can put 
my hand-wired gpsvario in checked-in luggage (no battery => no problem), with the power bank in my carry-on 
luggage as per airline requirements.
* Average current draw is ~160mA in gpsvario mode, ~300mA in wifi access point mode. Not
 optimized.
* Software uses [esp-idf with Arduino as a component](https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md), so we can take advantage of 
arduino-ESP32 code for the web server. 

### Build notes

#### Build environment
* Ubuntu 19.04 amdx64
* [esp-idf release tag v3.2.2](https://github.com/espressif/esp-idf/tree/v3.2.2)
* [arduino-esp32 release tag 1.03rc1] (https://github.com/espressif/arduino-esp32/tree/1.0.3-rc1) : Note that I have only retained the directories 
required for building this project
* xtensa-esp32-elf-gcc v5.2.0 (crosstool-ng-1.22.0-80-g6c4433a) 

#### Menuconfig

<img src="/docs/menuconfig_arduino.png" alt="menuconfig_arduino"/>
* We're only using the FS and Wifi libraries from the arduino-esp32 component

<img src="/docs/menuconfig_compiler.png" alt="menuconfig_compiler"/>
* We're using a mixture of C++ and C code for the project

<img src="/docs/menuconfig_partitiontable.png" alt="menuconfig_partitiontable"/>
* Run 'make flashfs' once to create and flash the spiffs partition image.

<img src="/docs/menuconfig_SPIFFS.png" alt="menuconfig_spiffs"/>

<img src="/docs/menuconfig_esp32_specific.png" alt="menuconfig_esp32_specific"/>
* 80MHz clock to minimize power consumption
* Main task stack size increased to 16384bytes to accommodate ESP32Webserver

<img src="/docs/menuconfig_PHY.png" alt="menuconfig_phy"/>
* Wifi transmit power reduced to 13dB from 20dB. This reduces the 
current spikes on wifi transmit bursts, so there's no need for a honking big capacitor on the 
ESP32 3.3V line. The reduced power isn't a problem for our application - if you're configuring or downloading data from
the gpsvario with a pc or smartphone, the units will likely be next to each other.

<img src="/docs/menuconfig_FreeRTOS.png" alt="menuconfig_freertos"/>
* FreeRTOS tick rate increased to 200Hz from 100Hz. This
allows a minimum tick delay of 5mS, which reduces the overhead of regular 
task yield during server data download etc.

##### Hardware
* I don't have a schematic for the project because I used  off-the-shelf modules/breakout boards
 (and one homebrew board for the audio amplifier). Have a look at the file
/main/config.h to find the signal connections from the ESP32 (#define pinXXX ) to various components. Use the 
ESP32 board USB 5V to supply power for the module boards (GPS, MPU9250, MS5611, audio amplifier, LCD), and
3.3V from the ESP32 VCC line for the 128Mb SPI flash.  Note that the LCD module PCB has a footprint for an
SOT23 type regulator. I soldered a 3.3V XC6203 regulator along with input and output bypass 10uF caps. All signal interfaces between the ESP32
and other components are at 3.3V level. 

* There are different versions of the 128x64 LCD module that need 
modifications to the initialization code (lcd bias, display orientation). So you may
need to tweak the code to get it to display at all or with the right orientation.
See lcd_init() in /ui/lcd7565.c.

* I added a 470uF 10V capacitor on the USB5V supply
before the power switch along with a 1A resettable polyfuse in the 5V supply line. Note that installing
a power switch requires breaking the 5V supply line from the microusb connector on the ESP32 breakout board.
The easiest way to do this is to desolder the schottky diode that is normally placed in the
5V supply line between the microusb connector 5V pin and the rest of the circuit. Connect the power switch inline in its place.

* I currently use a homebrew max4401 board for the audio amplifier, but you can 
use an inexpensive and compact NS8002 amplifier module from Aliexpress. Make sure to use 
a module with a shutdown(enable) pin to save power when not generating audio. If you're content with driving a piezo speaker, you can reduce power consumption 
by omitting the audio amplifier and driving the piezo directly from an ESP32 pin with square wave signals.


## Usage
* There are 4 user-interface buttons labeled as btnL(eft), btnM(iddle) and btnR(ight), plus btn0 (connected to gpio0). 
* For downloading binary data logs, put the gpsvario into server mode, connect to the WiFi access point 'ESP32GpsVario' and access the url 'http://192.168.4.1/datalog' via a web browser. The binary datalog file can contain a mix of high-speed IBG (imu+baro+gps) data samples, and normal GPS track logs. There is some sample software in the /offline directory for splitting the binary datalog into separate IBG and GPS datalogs, and for converting GPS logs into .gpx text files that you can load in Google Earth or other GPS track visualization software.
* For configuring the gpsvario, you can edit the user-configurable options on the LCD screen. In the options page, press the L or R buttons to select the option (o cursor). Press the M button to change
the option (* cursor). Now L and R will decrease/increase the value. Press the M button again to go back to the option select (o cursor). Changes are saved to the file options.txt in the onboard SPIFFS flash file system. 
Alternatively, put the gpsvario into server mode, access the url 'http://192.168.4.1' and download the options.txt file from the gpsvario. Edit it as required, and upload the file back to the gpsvario. This way you can keep different versions of the options.txt file on your laptop/smartphone for different sites or site conditions. To reset to 'factory defaults', just delete the options.txt file from the gpsvario using the webserver. It will be regenerated with default values the next time you power up the gpsvario.
* Use [xcplanner](https://xcplanner.appspot.com) to generate a route with waypoints in FormatGEO format as a *.wpt text file. Note that xcplanner does not specify waypoint radii in the FormatGEO file. You can edit the .wpt file to add the waypoint radius (in meters) at the end of a waypoint entry line. If the radius is not specified for a waypoint, the gpsvario will apply a user-configurable default waypoint radius. Upload the waypoint file to the gpsvario using the webpage upload file function. You can upload up to 7 route files and select one of them (or none) on-screen. If there
are no route files or you do not select one, the bearing-to-waypoint arrow and distance-to-waypoint field will display bearing and distance to the launch coordinates.

## Credits
* SPIFFS code - https://github.com/loboris/ESP32_spiffs_example
* Sine-wave generation with ESP32 DAC -  https://github.com/krzychb/dac-cosine
* Web server library - https://github.com/Pedroalbuquerque/ESP32WebServer
* Web server top level page handling, css style modified from  https://github.com/G6EJD/ESP32-ESP8266-File-Download-Upload-Delete-Stream-and-Directory
* MPU9250 initialization sequence modified from https://github.com/bolderflight/MPU9250/blob/master/MPU9250.cpp



