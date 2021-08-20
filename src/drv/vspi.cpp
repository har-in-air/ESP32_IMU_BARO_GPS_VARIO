#include "common.h"
#include "config.h"
#include "vspi.h"

spi_t * _vspi = NULL;

static int8_t 	_sclk;
static int8_t 	_miso;
static int8_t 	_mosi;
static uint32_t _div;
static uint32_t _freqHz;

static const char* TAG = "vspi";

void vspi_config(int pnSCLK, int pnMOSI, int pnMISO, int freqHz) {
	_freqHz = freqHz;
	_div = spiFrequencyToClockDiv(_freqHz);
	_vspi = spiStartBus(VSPI, _div, SPI_MODE0, SPI_MSBFIRST);
	if(!_vspi) {
		ESP_LOGE(TAG,"vspi StartBus error");
		return;
		}

   _sclk = pnSCLK;
   _miso = pnMISO;
   _mosi = pnMOSI;

   spiAttachSCK(_vspi, _sclk);
   spiAttachMOSI(_vspi, _mosi);
   if (_miso != -1) spiAttachMISO(_vspi, _miso);
   }


void vspi_setClockFreq(int freqHz) {
	_freqHz = freqHz;
	_div = spiFrequencyToClockDiv(_freqHz);
	spiSetClockDiv(_vspi, _div);
	}
	
	
	
