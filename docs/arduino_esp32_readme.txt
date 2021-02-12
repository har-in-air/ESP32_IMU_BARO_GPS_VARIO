Procedure for installing support for ESP-IDF projects with Arduino-ESP32 as a
component. This requires specific versions of ESP-IDF, Arduino-ESP32 and the
toolchain.

This assumes the installation directory path for esp-idf + arduino-esp32 projects
is INSTALL_DIR

Install esp-df v3.2 
-------------------
https://docs.espressif.com/projects/esp-idf/en/release-v3.2/get-started/index.html


cd INSTALL_DIR
git clone -b release/v3.2 --recursive https://github.com/espressif/esp-idf.git

cd esp-idf
chmod 755 add_path.sh
cd ..


Download and install the compiler toolchain v1.22.0-80
------------------------------------------------------
https://docs.espressif.com/projects/esp-idf/en/release-v3.2/get-started/linux-setup.html

cd INSTALL_DIR
tar -xzf ~/Downloads/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz


Export path for IDF and tools
-----------------------------

## Create a file setpath.sh in INSTALL_DIR with the following 3 lines :

export PATH="INSTALL_DIR/xtensa-esp32-elf/bin:$PATH"
export IDF_PATH="INSTALL_DIR/esp-idf"
$IDF_PATH/add_path.sh 

cd INSTALL_DIR
chmod 755 setpath.sh

When you want to work with an arduino-esp32+esp-idf project, 
open a new terminal window and run the script.

cd INSTALL_DIR
. ./setpath.sh

Better to do this rather than export the paths in .bashrc/.profile
as the ESP-IDF and tool versions used here are specific only to 
ESP-IDF + Arduino-ESP32 projects

Download and build the ESP32_IMU_BARO_GPS_VARIO project
-------------------------------------------------------

## Open new terminal window

cd INSTALL_DIR
git clone https://github.com/har-in-air/ESP32_IMU_BARO_GPS_VARIO.git
. ./setpath.sh
cd ESP32_IMU_BARO_GPS_VARIO
make menuconfig (change options as desired - serial port etc.)
make clean
make

## custom partition scheme, so this needs to be done just once
make erase_flash 

## Then you can just run these commands 
make clean
make
make flash
make monitor


Adding Arduino-ESP32 v1.04 as a component in a new project
----------------------------------------------------------
https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md

In project folder, 

mkdir -p components
cd components
git clone --recursive https://github.com/espressif/arduino-esp32.git arduino
cd arduino
git checkout tags/1.0.4
cd ../..
make menuconfig
	[*] Enable C++ exceptions
	Set serial port for flashing
	Component Config -> mbedTLS -> TLS Key Exchange Methods -> 
  	[*] Enable pre-shared-key ciphersuits
    [*] Enable PSK based ciphersuite modes


Fix the Arduino WiFi library compile errors
-------------------------------------------
https://github.com/espressif/arduino-esp32/issues/3760

Replace ETH.cpp and WiFiSTA.cpp with fixed versions 

e.g. from ESP32_IMU_BARO_GPS_VARIO components/arduino/libraries/WiFi/src


Add 3rd party Arduino libraries like WiFiManager
------------------------------------------------
Download the library zip and extract it to components/arduino/libraries


Removing unused Arduino libraries
---------------------------------
After verifying your project builds and executes as expected,
you can delete the arduino directories that aren't 
being used in your project. This will reduce build time after a 
'make clean', and reduce the size of project archives.
E.g. look at ESP32_IMU_BARO_GPS_VARIO/components/arduino


