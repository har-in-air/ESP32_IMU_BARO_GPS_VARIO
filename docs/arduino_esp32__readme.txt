1. Install esp-df v3.2 and associated toolchain as per
https://docs.espressif.com/projects/esp-idf/en/release-v3.2/get-started/index.html

In our example the installation directory is $HOME/esp32arduino

2. Execute the (modified as per your environment) path.sh in a terminal window to set up
the paths for esp-idf directories and the xtensa-elf-gcc tools. Now you can use the command
line tools to build and flash the code. Better to open a terminal and execute path.sh,
than to add the paths to your .bashrc or .profile files. This is to avoid conflict with 
other versions of esp-idf that are associated with different tool versions.

---- path.sh ---------

export PATH="$HOME/esp32arduino/xtensa-esp32-elf/bin:$PATH"
export IDF_PATH="$HOME/esp32arduino/esp-idf"
$IDF_PATH/add_path.sh

--------------------

3. navigate to one of the esp-idf example directories, and verify you can build and flash
the example using 'make menuconfig', 'make' and 'make flash'.

4. Add arduino-esp32 as a component in your project as per
https://github.com/espressif/arduino-esp32/blob/master/docs/esp-idf_component.md

In project folder, 

mkdir -p components
cd components
git clone --recursive https://github.com/espressif/arduino-esp32.git arduino
cd arduino
git checkout tags/1.0.4
cd ../..
make menuconfig

You can delete the arduino component library subdirectories that aren't being used in your
project.


