#ifndef SPIFLASH_H_
#define SPIFLASH_H_

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif
#if defined(FLASH_W25Q128)
   #define FLASH_SIZE_BYTES	16777216
#elif defined(FLASH_W25Q16)
   #define FLASH_SIZE_BYTES	2097152
#endif

#define FLASH_SECTOR_SIZE	4096


void 	spiflash_send(uint8_t out);
uint8_t spiflash_receive(void);
uint8_t spiflash_readByte(uint32_t address);
void 	spiflash_writeByte(uint32_t address, uint8_t wrByte);
uint8_t spiflash_getStatus(void);
void 	spiflash_chipErase(void);
void 	spiflash_sectorErase(uint32_t address);
void 	spiflash_globalUnprotect(void);
void 	spiflash_writeEnable(void);
void    spiflash_reset(void);
void 	spiflash_writeDisable(void);
uint16_t  spiflash_readID(void);
void 	spiflash_writeBuffer(uint32_t address, uint8_t* pBuffer, int nBytes);
void 	spiflash_readBuffer(uint32_t addr, uint8_t* pBuf, int nBytes);

#ifdef __cplusplus
}
#endif

#endif
