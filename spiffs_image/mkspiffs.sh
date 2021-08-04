python spiffsgen.py 0x10000 ./image spiffs_image.bin
python $IDF_PATH/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x180000 spiffs_image.bin
