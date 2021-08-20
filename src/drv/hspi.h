#ifndef HSPI_H_
#define HSPI_H_

#include <Arduino.h>

void hspi_config(int8_t pinsck, int8_t pinmiso, int8_t pinmosi, int8_t pinss, int freqHz);
void hspi_setHwCs(bool use);
void hspi_end();
void hspi_setFrequency(uint32_t freqHz);
void hspi_setClockDivider(uint32_t clockDiv);
uint32_t hspi_getClockDivider();
void hspi_setDataMode(uint8_t dataMode);
void hspi_setBitOrder(uint8_t bitOrder);
void hspi_write16(uint16_t data);
uint16_t hspi_transfer16(uint16_t data);
uint32_t hspi_transfer32(uint32_t data);
void hspi_transferBits(uint32_t data, uint32_t * out, uint8_t bits);
void hspi_writeBytes(uint8_t * data, uint32_t size);
uint8_t hspi_transfer(uint8_t data);
void hspi_write32(uint32_t data);
uint32_t hspi_transfer32(uint32_t data);
void hspi_write(uint8_t data);
void hspi_endTransaction();
void hspi_beginTransaction();


extern spi_t* _hspi;

#endif
