#include "common.h"
#include "config.h"
#include "hspi.h"

// If only using HSPI for driving the LCD, no need to reserve gpio for MISO
#define USE_HMISO false

spi_t * _hspi = NULL;
static bool 	_use_hw_ss = false;
static int8_t 	_pinsck = -1;
static int8_t 	_pinmiso = -1;
static int8_t 	_pinmosi = -1;
static int8_t 	_pinss = -1;
static uint32_t _div = 0;
static uint32_t _freqHz = 1000000;
static bool _inTransaction = false;

static const char* TAG = "hspi";

void hspi_config(int8_t pinsck, int8_t pinmiso, int8_t pinmosi, int8_t pinss, int freqHz){
    if(_hspi) {
        return;
        }
    if(!_div) {
    	_freqHz = freqHz;
        _div = spiFrequencyToClockDiv(_freqHz);
        }

    _hspi = spiStartBus(HSPI, _div, SPI_MODE0, SPI_MSBFIRST);
    if(!_hspi) {
        return;
        }

    if(pinsck == -1 && pinmiso == -1 && pinmosi == -1 && pinss == -1) {
        _pinsck = 14;
        _pinmiso = 12;
        _pinmosi = 13;
        _pinss = 15;
        } 
    else {
        _pinsck = pinsck;
        _pinmiso = pinmiso;
        _pinmosi = pinmosi;
        _pinss = pinss;
        }
    spiAttachSCK(_hspi, _pinsck);
#if USE_HMISO    
    spiAttachMISO(_hspi, _pinmiso);
#endif    
    spiAttachMOSI(_hspi, _pinmosi);
    if (_pinss == -1) {
        spiSSDisable(_hspi);
        spiDetachSS(_hspi, _pinss);
        _use_hw_ss = false;
        }
    else {
        spiAttachSS(_hspi, 0, _pinss);
        spiSSEnable(_hspi);
        _use_hw_ss = true;
        }
    }

void hspi_end(){
    if(!_hspi) {
        return;
        }
    spiDetachSCK(_hspi, _pinsck);
    //spiDetachMISO(_hspi, _pinmiso);
    spiDetachMOSI(_hspi, _pinmosi);
    hspi_setHwCs(false);
    spiStopBus(_hspi);
    _hspi = NULL;
    }

void hspi_setHwCs(bool use){
    if(use && !_use_hw_ss) {
        spiAttachSS(_hspi, 0, _pinss);
        spiSSEnable(_hspi);
        } 
    else if(_use_hw_ss) {
        spiSSDisable(_hspi);
        spiDetachSS(_hspi, _pinss);
        }
    _use_hw_ss = use;
    }

void hspi_setFrequency(uint32_t freqHz){
    //check if last freq changed
    uint32_t cdiv = spiGetClockDiv(_hspi);
    if(_freqHz != freqHz || _div != cdiv) {
        _freqHz = freqHz;
        _div = spiFrequencyToClockDiv(_freqHz);
        spiSetClockDiv(_hspi, _div);
        }
    }

void hspi_setClockDivider(uint32_t clockDiv){
    _div = clockDiv;
    spiSetClockDiv(_hspi, _div);
    }

uint32_t hspi_getClockDivider(){
    return spiGetClockDiv(_hspi);
    }

void hspi_setDataMode(uint8_t dataMode){
    spiSetDataMode(_hspi, dataMode);
    }

void hspi_setBitOrder(uint8_t bitOrder){
    spiSetBitOrder(_hspi, bitOrder);
    }

void hspi_beginTransaction(){
    spiTransaction(_hspi, _div, SPI_MODE0, SPI_MSBFIRST);
    _inTransaction = true;
    }

void hspi_endTransaction(){
    if(_inTransaction){
        _inTransaction = false;
        spiEndTransaction(_hspi);
        }
    }

void hspi_write(uint8_t data){
    if(_inTransaction){
        return spiWriteByteNL(_hspi, data);
        }
    spiWriteByte(_hspi, data);
    }

uint8_t hspi_transfer(uint8_t data){
    if(_inTransaction){
        return spiTransferByteNL(_hspi, data);
        }
    return spiTransferByte(_hspi, data);
    }

void hspi_write16(uint16_t data){
    if(_inTransaction){
        return spiWriteShortNL(_hspi, data);
        }
    spiWriteWord(_hspi, data);
    }

uint16_t hspi_transfer16(uint16_t data){
    if(_inTransaction){
        return spiTransferShortNL(_hspi, data);
        }
    return spiTransferWord(_hspi, data);
    }   

void hspi_write32(uint32_t data){
    if(_inTransaction){
        return spiWriteLongNL(_hspi, data);
        }
    spiWriteLong(_hspi, data);
    }

uint32_t hspi_transfer32(uint32_t data){
    if(_inTransaction){
        return spiTransferLongNL(_hspi, data);
        }
    return spiTransferLong(_hspi, data);
    }

void hspi_transferBits(uint32_t data, uint32_t * out, uint8_t bits){
    if(_inTransaction){
        return spiTransferBitsNL(_hspi, data, out, bits);
        }
    spiTransferBits(_hspi, data, out, bits);
    }

void hspi_writeBytes(uint8_t * data, uint32_t size){
    if(_inTransaction){
        return spiWriteNL(_hspi, data, size);
        }
    spiSimpleTransaction(_hspi);
    spiWriteNL(_hspi, data, size);
    spiEndTransaction(_hspi);
    }


/**
 * @param data uint8_t * data buffer. can be NULL for Read Only operation
 * @param out  uint8_t * output buffer. can be NULL for Write Only operation
 * @param size uint32_t
 */
void hspi_transferBytes(uint8_t * data, uint8_t * out, uint32_t size){
    if(_inTransaction){
        return spiTransferBytesNL(_hspi, data, out, size);
        }
    spiTransferBytes(_hspi, data, out, size);
    }


