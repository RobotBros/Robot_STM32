/*******************************************************************************
 * FileName：Robot_Flash.c
 * Function:   Robot spi Flash read and write
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	

#include "Robot_Flash.h"

/*Array Size define-----------------------------------------------------------*/
#define ROBOT_ACTION_BYE_LEN    (1000) 				// Flash opration length
/*static variable define----------------------------------------------------------*/ 
static u16 *FlashReg_Addr[ROBOT_ACTION_BYE_LEN];
//static u8   Flash_BUFF[16];
//static u16 Checksum = 0;

static u16 CheckSumFault = 0;


/*******************************************************************************
 * Routine     : Robot_Flash_Init
 * Function    : Robot Flash Initialization
 ******************************************************************************/
void Robot_Flash_Init(void)
{
	u32 tmp32 = 0;
	//u8   cnt = 0;

	for(tmp32=0;tmp32<ROBOT_ACTION_BYE_LEN;tmp32++)
	{FlashReg_Addr[tmp32] = 0;}

	FlashReg_Addr[0] = (u16*)&CheckSumFault;
	
	Robot_Flash_SPI1_Init();

	/*0x000H-0x00FH Flash Gear Action Parameter------------------------------------*/
	//Robot_Flash_Read(Flash_BUFF,u32 ReadAddr,16);  			
	//for(cnt=0;cnt<16;cnt++)    Checksum+=Flash_BUFF[cnt];
	/*Data to Action */
}

/*******************************************************************************
 * Routine     : Robot_Flash_SPI1_Init
 * Function    : Robot Flash SPI1 Initialization
 ******************************************************************************/
void Robot_Flash_SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,GPIO_Pin_5 |GPIO_Pin_6 |GPIO_Pin_7);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	//设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;					//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//定义波特率预分频的值:波特率预分频值为2  36/2=18MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); 										//使能SPI外设
	
	//SPI1_Read_Write_Byte(0xff);									//启动传输	
}   

/*******************************************************************************
 * Routine   : Robot_FLASH_Out
 * Function  : Flash out parm
 ******************************************************************************/
u16 Robot_FLASH_Out(u8 regist)
{  
	if(regist < 50)	return (*FlashReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine   : SPI1_ReadWriteByte
 * Function  : Read and Write SPI1 
 		      TxData write data and return data 1 byte ,wait time funtion
 ******************************************************************************/
u8 SPI1_Read_Write_Byte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry > OPERATION_TIMES)return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData); 
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry > OPERATION_TIMES)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); 				    
}

/*******************************************************************************
 * Routine   : Robot_Flash_ReadID
 * Function  : Read Robot Flash ID = OxEF14,wait time funtion
 ******************************************************************************/
u16 Robot_Flash_ReadID(void)
{
	u16 id = 0;	  
			    
	SPI1_Read_Write_Byte(0x90);				// Send read ID command
	SPI1_Read_Write_Byte(0x00); 	    
	SPI1_Read_Write_Byte(0x00); 	    
	SPI1_Read_Write_Byte(0x00); 	 			   
	id|=SPI1_Read_Write_Byte(0xFF)<<8;  
	id|=SPI1_Read_Write_Byte(0xFF);	 
		    
	return id;
}   

/*******************************************************************************
 * Routine   : Robot_Flash_Read_SR
 * Function  : Read Robot Flash Status register,wait time funtion
 *BIT7  6   5   4   3   2   1   0
 *SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *SPR:默认0,状态寄存器保护位,配合WP使用
 *TB,BP2,BP1,BP0:FLASH区域写保护设置
 *WEL:写使能锁定
 *BUSY:忙标记位(1,忙;0,空闲)
 *默认:0x00
 ******************************************************************************/
u8 Robot_Flash_Read_SR(void)   
{  
	u8 temp8=0;   
	
	SPI1_Read_Write_Byte(W25X_ReadStatusReg);    	//发送读取状态寄存器命令    
	temp8 = SPI1_Read_Write_Byte(0Xff);             		//读取一个字节  
	return temp8;   
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Write_SR
 * Function  :  Write Flash Status register,wait time funtion
 *BIT7  6   5   4   3   2   1   0
 *SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *Only SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2) can write
 ******************************************************************************/
void Robot_Flash_Write_SR(u8 sr)   
{    
	SPI1_Read_Write_Byte(W25X_WriteStatusReg);   		//发送写取状态寄存器命令    
	SPI1_Read_Write_Byte(sr);               			  	//写入一个字节    	      
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Write_Enable
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Write_Enable(void)   
{
    	SPI1_Read_Write_Byte(W25X_WriteEnable);      		//发送写使能    	      
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Write_Disable
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Write_Disable(void)   
{  
   	SPI1_Read_Write_Byte(W25X_WriteDisable);     		//发送写禁止指令       	      
} 	

/*******************************************************************************
 * Routine   : Robot_Flash_PowerDown
 * Function  :  Write Flash Power down register, wait time funtion
 ******************************************************************************/
void Robot_Flash_PowerDown(void)   
{ 
    SPI1_Read_Write_Byte(W25X_PowerDown);        		//发送掉电命令  	      
}   

/*******************************************************************************
 * Routine   : Robot_Flash_WAKEUP
 * Function  :  Write Flash Power up register, wait time funtion
 ******************************************************************************/
void Robot_Flash_WAKEUP(void)   
{  
    	SPI1_Read_Write_Byte(W25X_ReleasePowerDown);   
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Wait_Busy
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Wait_Busy(void)   
{   
	//while (Robot_Flash_Write_SR(0x01)==0x01);   	// 等待BUSY位清空 ?
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Erase_Chip
 * Function  :  Erase Flash whole chip,  wait time funtion S 
 ******************************************************************************/
void Robot_Flash_Erase_Chip(void)   
{                       
	Robot_Flash_Write_Enable();
    	Robot_Flash_Wait_Busy();   
    	SPI1_Read_Write_Byte(W25X_ChipErase);        		//发送片擦除命令  
	Robot_Flash_Wait_Busy();   				   		//等待芯片擦除结束
}   

/*******************************************************************************
 * Routine   : Robot_Flash_Erase_Sector
 * Function  :  Erase Flash chip sector,  wait time funtion S 
 *擦除一个扇区
 *Dst_Addr:扇区地址 0~511 for w25x16
 *擦除一个山区的最少时间:150ms
 ******************************************************************************/
void Robot_Flash_Erase_Sector(u32 Dst_Addr)   
{   
	Dst_Addr*=4096;
	
    	Robot_Flash_Write_Enable();                
    	Robot_Flash_Wait_Busy();   
    	SPI1_Read_Write_Byte(W25X_SectorErase);      		//发送扇区擦除指令 
    	SPI1_Read_Write_Byte((u8)((Dst_Addr)>>16));  	//发送24bit地址    
    	SPI1_Read_Write_Byte((u8)((Dst_Addr)>>8));   
    	SPI1_Read_Write_Byte((u8)Dst_Addr);    	      
    	Robot_Flash_Wait_Busy();   				  
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Read
 * Function  :  Read Flash data, wait time funtion
 * Read Flash Fix Length data 
 * pBuffer:数据存储区
 * ReadAddr:开始读取的地址(24bit)
 * Len:要读取的字节数(最大65535)
 ******************************************************************************/
void  Robot_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 Len)   
{ 
	u16 cnt16 = 0;    
	
    	SPI1_Read_Write_Byte(W25X_ReadData);         		//发送读取命令   
    	SPI1_Read_Write_Byte((u8)((ReadAddr)>>16));  	//发送24bit地址    
    	SPI1_Read_Write_Byte((u8)((ReadAddr)>>8));   
    	SPI1_Read_Write_Byte((u8)ReadAddr);   
    	for(cnt16=0;cnt16<Len;cnt16++)
	{ 
        	pBuffer[cnt16]=SPI1_Read_Write_Byte(0XFF);   //循环读数  
    	}	      
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Read
 * Function  :  Read Flash data, wait time funtion
 * Write Flash Fix Length data 
 * pBuffer:数据存储区
 * ReadAddr:开始写入的地址(24bit)
 * Len:要读取的字节数(最大65535),该数不应该超过该页的剩余字节数
 ******************************************************************************/
void Robot_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 Len)
{
	u16 cnt16 = 0;
	
    	Robot_Flash_Write_Enable();                  
    	SPI1_Read_Write_Byte(W25X_PageProgram);      			//发送写页命令   
    	SPI1_Read_Write_Byte((u8)((WriteAddr)>>16)); 		//发送24bit地址    
    	SPI1_Read_Write_Byte((u8)((WriteAddr)>>8));   
    	SPI1_Read_Write_Byte((u8)WriteAddr);   
   	for(cnt16=0;cnt16<Len;cnt16++)SPI1_Read_Write_Byte(pBuffer[cnt16]);
	Robot_Flash_Wait_Busy();					   		//等待写入结束
} 

