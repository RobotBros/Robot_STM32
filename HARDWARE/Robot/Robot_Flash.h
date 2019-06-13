#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

#define OPERATION_TIMES	200			// Read and write SPI1 times

//W25X系列/Q系列芯片列表	   
#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16

//指令表
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg		0x05 
#define W25X_WriteStatusReg		0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 
 				  	    													  
void Robot_Flash_Init(void);
void Robot_Flash_SPI1_Init(void);
u16 Robot_FLASH_Out(u8 regist);
u8 SPI1_Read_Write_Byte(u8 TxData);
u16 Robot_Flash_ReadID(void);
u8 Robot_Flash_Read_SR(void);
void Robot_Flash_Write_SR(u8 sr);
void Robot_Flash_Write_Enable(void);
void Robot_Flash_Write_Disable(void);
void Robot_Flash_PowerDown(void);
void Robot_Flash_WAKEUP(void);
void Robot_Flash_Wait_Busy(void);
void Robot_Flash_Erase_Chip(void);
void Robot_Flash_Erase_Sector(u32 Dst_Addr);
void  Robot_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 Len);
void Robot_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 Len);

		 
#endif

