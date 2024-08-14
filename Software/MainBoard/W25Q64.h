#include "config.h"
#include "stm32f10x.h"


//********************************************************************************
#ifndef	_W25Q64_H_
#define	_W25Q64_H_

#include "SPI.h"

#define W25Q64_WRITE_ENABLE				0x06
#define W25Q64_WRITE_DISABLE				0x04
#define W25Q64_READ_STATUS_REGISTER_1			0x05
#define W25Q64_READ_STATUS_REGISTER_2			0x35
#define W25Q64_WRITE_STATUS_REGISTER			0x01
#define W25Q64_PAGE_PROGRAM				0x02
#define W25Q64_QUAD_PAGE_PROGRAM			0x32
#define W25Q64_BLOCK_ERASE_64KB				0xD8
#define W25Q64_BLOCK_ERASE_32KB				0x52
#define W25Q64_SECTOR_ERASE_4KB				0x20
#define W25Q64_CHIP_ERASE				0xC7
#define W25Q64_ERASE_SUSPEND				0x75
#define W25Q64_ERASE_RESUME				0x7A
#define W25Q64_POWER_DOWN				0xB9
#define W25Q64_HIGH_PERFORMANCE_MODE			0xA3
#define W25Q64_CONTINUOUS_READ_MODE_RESET		0xFF
#define W25Q64_RELEASE_POWER_DOWN_HPM_DEVICE_ID		0xAB
#define W25Q64_MANUFACTURER_DEVICE_ID			0x90
#define W25Q64_READ_UNIQUE_ID				0x4B
#define W25Q64_JEDEC_ID					0x9F
#define W25Q64_READ_DATA				0x03
#define W25Q64_FAST_READ				0x0B
#define W25Q64_FAST_READ_DUAL_OUTPUT			0x3B
#define W25Q64_FAST_READ_DUAL_IO			0xBB
#define W25Q64_FAST_READ_QUAD_OUTPUT			0x6B
#define W25Q64_FAST_READ_QUAD_IO			0xEB
#define W25Q64_OCTAL_WORD_READ_QUAD_IO			0xE3
#define W25Q64_GLOBAL_BLOCK_UNLOCK			0x98
#define W25Q64_DUMMY_BYTE				0xFF

void W25Q64_ReadID(uint8_t *MID, uint16_t *DID);
void W25Q64_Write_Sector(uint32_t Address);
void W25Q64_Read_Sector(uint32_t Address);
void W25Q64_Print_Buffer(void);
#endif	//_W25Q64_H_

