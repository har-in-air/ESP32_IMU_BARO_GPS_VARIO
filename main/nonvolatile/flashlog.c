#include "common.h"
#include "config.h"
#include "options.h"
#include "flashlog.h"

uint32_t          FlashLogFreeAddress  = 0;
FLASHLOG_IBG_RECORD   FlashLogIBGRecord;
FLASHLOG_GPS_RECORD   FlashLogGPSRecord;
SemaphoreHandle_t FlashLogMutex = NULL;

#define TAG "flashlog"


void flashlog_erase(uint32_t untilAddress) {
	spiflash_globalUnprotect();
   if (untilAddress == 0) untilAddress = FLASH_SIZE_BYTES; // manually force erase of entire chip
   uint32_t lastSectorAddress = untilAddress & 0xFFFFF000; // sector size = 4096
	for (uint32_t sectorAddress = 0; sectorAddress <= lastSectorAddress; sectorAddress += 4096) {
		spiflash_sectorErase(sectorAddress);
		delayMs(50);
		}
	ESP_LOGI(TAG,"Erased");
   FlashLogFreeAddress = 0;
   }

	
int flashlog_init(void ) {
	uint16_t flashID = spiflash_readID();
	if (flashID != 0xEF17) {
		ESP_LOGE(TAG, "Winbond W25Q128FVSG ID [expected EF17] = %04X", flashID);
		return -1;
		}
	spiflash_globalUnprotect();
  	FlashLogMutex = xSemaphoreCreateMutex();
   if (FlashLogMutex == NULL) {
   	ESP_LOGE(TAG, "Error creating FlashLogMutex");
   	return -2;
   	}
   FlashLogFreeAddress = flashlog_getFreeAddress();
	ESP_LOGI(TAG,"FlashLogFreeAddress = %08d", FlashLogFreeAddress);
	return 0;
	}


int flashlog_isEmpty() {
   uint32_t dw;
   spiflash_readBuffer(0, (uint8_t*)&dw, sizeof(uint32_t));
   return (dw == 0xFFFF) ? 1 : 0;
   }


// if there is only an I record, size of record = IBGHDR + I
// if there is a B record, size of record = IBGHDR + I + B
// if there is a G record, size of record = IBGHDR + I + B + G even if B record is invalid

int flashlog_getNumIBGRecords() {
   uint32_t addr = 0;
   uint32_t maxAddr = FLASH_SIZE_BYTES - sizeof(FLASHLOG_IBG_RECORD); 
   int numRecords = 0;
   while (1) {
      if (addr >= maxAddr) break; // flash is full
      IBG_HDR hdr;
      hdr.magic = 0;
      spiflash_readBuffer(addr, (uint8_t*)&hdr, sizeof(IBG_HDR));
      if (hdr.magic == 0xFFFF) break; // found free address
      addr += (sizeof(IBG_HDR) + sizeof(I_RECORD));
      if (hdr.baroFlags || hdr.gpsFlags) addr += sizeof(B_RECORD);
      if (hdr.gpsFlags) addr += sizeof(G_RECORD);
      numRecords++;
	   if ((numRecords % 100) == 0) delayMs(10); // yield for task watchdog
      }
   return numRecords;
   }


uint32_t flashlog_getFreeAddress() {
   uint32_t loAddr = 0;
   uint32_t hiAddr = FLASH_SIZE_BYTES - 4;
   uint32_t midAddr = ((loAddr+hiAddr)/2)&0xFFFFFFFE;
   while (hiAddr > loAddr+4) {
      uint32_t dw;
      spiflash_readBuffer(midAddr, (uint8_t*)&dw, sizeof(uint32_t));
      if (dw == 0xFFFFFFFF) {
         hiAddr = midAddr;
         }
      else {
         loAddr = midAddr;
         }
      midAddr = ((loAddr+hiAddr)/2)&0xFFFFFFFE;
      }
   ESP_LOGI(TAG,"Lo %d mid %d hi %d", loAddr, midAddr, hiAddr);
   return midAddr;
   }

    
// ensure this function is called within a mutex take/give as there are multiple tasks
// updating the log record
int flashlog_writeIBGRecord(FLASHLOG_IBG_RECORD* pRecord) {
	if (FlashLogFreeAddress > (FLASHLOG_MAX_ADDR - sizeof(FLASHLOG_IBG_RECORD))) {
      return -1;
      }
   int numBytes = sizeof(IBG_HDR) + sizeof(I_RECORD);
   if (pRecord->hdr.baroFlags || pRecord->hdr.gpsFlags) numBytes += sizeof(B_RECORD);
   if (pRecord->hdr.gpsFlags) numBytes += sizeof(G_RECORD);

	spiflash_writeBuffer(FlashLogFreeAddress, (uint8_t*)pRecord, numBytes);
	FlashLogFreeAddress += numBytes;
   return 0;
	}



int flashlog_readIBGRecord(uint32_t addr, FLASHLOG_IBG_RECORD* pRecord) {
   IBG_HDR hdr;
   spiflash_readBuffer(addr, (uint8_t*)&hdr, sizeof(IBG_HDR));
   if (hdr.magic != FLASHLOG_IBG_MAGIC) {
      ESP_LOGE(TAG, "error flash log header does not have magic id");
      return -1;
      }
   int numBytes = sizeof(IBG_HDR) + sizeof(I_RECORD);
   if (hdr.baroFlags || hdr.gpsFlags) numBytes += sizeof(B_RECORD);
   if (hdr.gpsFlags) numBytes += sizeof(G_RECORD);
	spiflash_readBuffer(addr, (uint8_t*)pRecord, numBytes);
   return 0;
	}


// this function does not have to be called within a mutex take/give as
// only gps_task updates and writes the log record
int flashlog_writeGPSRecord(FLASHLOG_GPS_RECORD* pRecord) {
	if (FlashLogFreeAddress > (FLASHLOG_MAX_ADDR - sizeof(FLASHLOG_GPS_RECORD))) {
      return -1;
      }
   int numBytes = sizeof(FLASHLOG_GPS_RECORD);
	spiflash_writeBuffer(FlashLogFreeAddress, (uint8_t*)pRecord, numBytes);
	FlashLogFreeAddress += numBytes;
   return 0;
	}

