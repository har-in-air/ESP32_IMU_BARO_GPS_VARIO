#include "common.h"
#include "config.h"
#include "vspi.h"
#include "spiflash.h"

#define TAG "spiflash"


#define SPIFLASH_BUSY() 	(spiflash_getStatus() & 0x01)

inline void spiflash_writeEnable(void) {
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x06);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   }


/// Send write-enable command prior to writing data
inline void spiflash_writeDisable(void) {
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x04);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   }	

/// For allowing writes to the flash, the chip needs to be
/// globally unprotected (done during initialization on system power up)
void spiflash_globalUnprotect(void) {
   while (SPIFLASH_BUSY());
   spiflash_writeEnable();
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x01); // Write Status register
   spiTransferByteNL(_vspi, 0x00); // enable all blocks for write
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   }


/// Erase a 4kbyte sector, 18mS
/// @param address within the block
void spiflash_sectorErase(uint32_t address){
   while (SPIFLASH_BUSY());
   spiflash_writeEnable();
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x20); // 4Kbyte sector
   spiTransferByteNL(_vspi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)(address&0xff));
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   while (SPIFLASH_BUSY());
   }


/// Erase the entire chip, 35mS
void spiflash_chipErase(void) {
   while (SPIFLASH_BUSY());
   spiflash_writeEnable();
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0xC7);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   while (SPIFLASH_BUSY());
   }


/// Check status of last command
uint8_t  spiflash_getStatus(void) {
   uint8_t rcvByte;
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x05);
   rcvByte = spiTransferByteNL(_vspi, 0);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   return rcvByte;
   }



/// Write a single byte to a 24bit address 7uS
/// Block must be already erased
void spiflash_writeByte(uint32_t address, uint8_t wrByte) {
   while (SPIFLASH_BUSY());
   spiflash_writeEnable();
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x02);
   spiTransferByteNL(_vspi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)(address&0xff));
   spiTransferByteNL(_vspi, wrByte);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   }

/// Read a single byte from 24-bit address, high-speed mode upto 104MHz
uint8_t spiflash_readByte(uint32_t address) {
   uint8_t rdByte;
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   spiTransferByteNL(_vspi, 0x03);
   spiTransferByteNL(_vspi, (uint8_t)((address>>16)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)((address>>8)&0xff));
   spiTransferByteNL(_vspi, (uint8_t)(address&0xff));
   rdByte =  spiTransferByteNL(_vspi, 0);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);
   return rdByte;
   }

	
void spiflash_writeBuffer(uint32_t addr, uint8_t* pBuf, int nBytes) {
   int nPageFreeBytes,nP0Bytes, nP1Bytes;
   nPageFreeBytes = (int)(256 - (addr & 0x000000ff));
   if (nBytes <= nPageFreeBytes) {
		nP0Bytes = nBytes;
		nP1Bytes = 0;
		}
	else {
		nP0Bytes = nPageFreeBytes;
		nP1Bytes = nBytes - nP0Bytes;
		}

   while (SPIFLASH_BUSY());
   spiflash_writeEnable();
   spiSimpleTransaction(_vspi);
   FLASH_CS_LO();
   uint8_t cmd[] = {0x02, (uint8_t)((addr>>16)&0xff), (uint8_t)((addr>>8)&0xff),(uint8_t)(addr&0xff)};
   spiTransferBytesNL(_vspi, cmd, NULL, 4);
   spiTransferBytesNL(_vspi, pBuf, NULL, nP0Bytes);
   FLASH_CS_HI();
   spiEndTransaction(_vspi);

	if (nP1Bytes) {
	   addr >>= 8;
		addr++;
		addr <<= 8;
	   while (SPIFLASH_BUSY());
		spiflash_writeEnable();
		spiSimpleTransaction(_vspi);
      FLASH_CS_LO();
      uint8_t cmd[] = {0x02, (uint8_t)((addr>>16)&0xff), (uint8_t)((addr>>8)&0xff),(uint8_t)(addr&0xff)};
      spiTransferBytesNL(_vspi, cmd, NULL, 4);
      pBuf += nP0Bytes;
      spiTransferBytesNL(_vspi, pBuf, NULL, nP1Bytes);
		FLASH_CS_HI();
		spiEndTransaction(_vspi);
		}
	}


void spiflash_readBuffer(uint32_t addr, uint8_t* pBuf, int nBytes) {
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_vspi);
	FLASH_CS_LO();
	spiTransferByteNL(_vspi, 0x0B);
	spiTransferByteNL(_vspi, (uint8_t)((addr>>16)&0xff));
	spiTransferByteNL(_vspi, (uint8_t)((addr>>8)&0xff));
	spiTransferByteNL(_vspi, (uint8_t)(addr&0xff));
	spiTransferByteNL(_vspi, 0x00);

   spiTransferBytesNL(_vspi, NULL, pBuf, nBytes);

	FLASH_CS_HI();
	spiEndTransaction(_vspi);
	return;
	}	


// Winbond W25Q16BV ID = 0xEF14
// Winbond W25Q128FVSG ID = 0xEF17

uint16_t spiflash_readID(void) {
   uint8_t manufID, devID;
   while (SPIFLASH_BUSY());
   spiSimpleTransaction(_vspi);
	FLASH_CS_LO();
	spiTransferByteNL(_vspi, 0x90);
	spiTransferByteNL(_vspi, 0);
	spiTransferByteNL(_vspi, 0);
	spiTransferByteNL(_vspi, 0);
   manufID = spiTransferByteNL(_vspi, 0);
   devID = spiTransferByteNL(_vspi, 0);
   FLASH_CS_HI();
	spiEndTransaction(_vspi);
   return  (((uint16_t)manufID)<<8 | (uint16_t)devID);
   }

	
