#include "common.h"
#include "config.h"
#include "hspi.h"


spi_t * _hspi = NULL;

static int8_t 	_sclk;
static int8_t 	_miso;
static int8_t 	_mosi;
static uint32_t _div;
static uint32_t _freqHz;

#define TAG "hspi"


void hspi_config(int pnSCLK, int pnMOSI, int pnMISO, int freqHz) {
	_freqHz = freqHz;
	_div = spiFrequencyToClockDiv(_freqHz);
	_hspi = spiStartBus(HSPI, _div, SPI_MODE0, SPI_MSBFIRST);
	if(!_hspi) {
		ESP_LOGE(TAG,"hspi StartBus error");
		return;
		}

   _sclk = pnSCLK;
   _mosi = pnMOSI;
   _miso = pnMISO;

   spiAttachSCK(_hspi, _sclk);
   spiAttachMOSI(_hspi, _mosi);
   if (_miso != -1) spiAttachMISO(_hspi, _miso);
   }


void hspi_setClockFreq(int freqHz) {
	_freqHz = freqHz;
	_div = spiFrequencyToClockDiv(_freqHz);
	spiSetClockDiv(_hspi, _div);
	}
	





