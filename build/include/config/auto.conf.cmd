deps_config := \
	/home/hari/esp32/esp-idf-release-v3.3/components/app_trace/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/aws_iot/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/bt/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/driver/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/efuse/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp32/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp_adc_cal/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp_event/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp_http_client/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp_http_server/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/esp_https_ota/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/espcoredump/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/ethernet/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/fatfs/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/freemodbus/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/freertos/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/heap/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/libsodium/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/log/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/lwip/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/mbedtls/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/mdns/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/mqtt/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/nvs_flash/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/openssl/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/pthread/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/spi_flash/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/tcpip_adapter/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/unity/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/vfs/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/wear_levelling/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/wifi_provisioning/Kconfig \
	/home/hari/esp32/esp-idf-release-v3.3/components/app_update/Kconfig.projbuild \
	/home/hari/esp32/ESP32_IMU_BARO_GPS_VARIO/components/arduino/Kconfig.projbuild \
	/home/hari/esp32/esp-idf-release-v3.3/components/bootloader/Kconfig.projbuild \
	/home/hari/esp32/esp-idf-release-v3.3/components/esptool_py/Kconfig.projbuild \
	/home/hari/esp32/ESP32_IMU_BARO_GPS_VARIO/main/Kconfig.projbuild \
	/home/hari/esp32/esp-idf-release-v3.3/components/partition_table/Kconfig.projbuild \
	/home/hari/esp32/esp-idf-release-v3.3/Kconfig

include/config/auto.conf: \
	$(deps_config)

ifneq "$(IDF_TARGET)" "esp32"
include/config/auto.conf: FORCE
endif
ifneq "$(IDF_CMAKE)" "n"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
