1. Install esp-df v3.2 and associated toolchain, see

https://docs.espressif.com/projects/esp-idf/en/release-v3.2/get-started/index.html

In our example the installation directory is $HOME/esp32arduino

cd $HOME/esp32arduino
git clone -b release/v3.2 --recursive https://github.com/espressif/esp-idf.git

cd esp-idf
chmod 755 add_path.sh
cd ..

Download the toolchain, then extract it to $HOME/esp32arduino

tar -xzf ~/Downloads/xtensa-esp32-elf-linux64-1.22.0-80-g6c4433a-5.2.0.tar.gz

2. Execute the following path.sh in a terminal window to set up
the paths for esp-idf directories and the xtensa-elf-gcc tools. Now you can use the command
line tools to build and flash the code. Better to open a terminal and execute path.sh,
than to add the paths to your .bashrc or .profile files. This is to avoid conflict with 
other versions of esp-idf that are associated with different tool versions.

---- path.sh ---------

export PATH="$HOME/esp32arduino/xtensa-esp32-elf/bin:$PATH"
export IDF_PATH="$HOME/esp32arduino/esp-idf"
$IDF_PATH/add_path.sh

--------------------

4. Add arduino-esp32 as a component in your project, see

https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md

In project folder, 

mkdir -p components
cd components
git clone --recursive https://github.com/espressif/arduino-esp32.git arduino
cd arduino
git checkout tags/1.0.4
cd ../..
make menuconfig

Fix the Arduino WiFi library sources, see

https://github.com/espressif/arduino-esp32/issues/3760

You can delete the arduino component library directories that aren't being used in your
project.


