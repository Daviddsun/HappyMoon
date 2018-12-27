#ifndef __FLASH_H
#define __FLASH_H
#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include "Task.h"

#define STM32_FLASH_BASE  0X080E0004 

#define ADDR_FLASH_SECTOR_0     ((u32)0x08000000) 
#define ADDR_FLASH_SECTOR_1     ((u32)0x08004000)  
#define ADDR_FLASH_SECTOR_2     ((u32)0x08008000)  
#define ADDR_FLASH_SECTOR_3     ((u32)0x0800C000) 
#define ADDR_FLASH_SECTOR_4     ((u32)0x08010000) 
#define ADDR_FLASH_SECTOR_5     ((u32)0x08020000) 
#define ADDR_FLASH_SECTOR_6     ((u32)0x08040000) 	 
#define ADDR_FLASH_SECTOR_7     ((u32)0x08060000)  
#define ADDR_FLASH_SECTOR_8     ((u32)0x08080000)  
#define ADDR_FLASH_SECTOR_9     ((u32)0x080A0000) 	 
#define ADDR_FLASH_SECTOR_10    ((u32)0x080C0000) 
#define ADDR_FLASH_SECTOR_11    ((u32)0x080E0000) 


u32 STMFLASH_ReadWord(u32 faddr);
uint16_t STMFLASH_GetFlashSector(u32 addr);
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite);
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead);

#endif


