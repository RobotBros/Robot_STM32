/*******************************************************************************
 * FileName��Robot_ESP32.c
 * Function:  Robot Command function - SPI Slave, Host is ESP32
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_ESP32.h"

/*static variable define----------------------------------------------------------*/ 
static u8 Esp32_tx_buf[ESP_TX_SIZE];
static u8 Esp32_rx_buf[ESP_RX_SIZE];
static u8 TmsFlag_ESPCom_handle = 0;
static u8 RXD_Done_Flag = 0;

/*******************************************************************************
 * Routine     : Robot_ESP32_Init
 * Function    : Robot ESP32 Interface Initialization
 ******************************************************************************/
void Robot_ESP32_Init(void)
{	
	Robot_ESP32_SPI2_Init();
}

/*******************************************************************************
 * Routine     : Robot_ESP32_SPI2_Init
 * Function    : Robot ESP32 SPI2 Initialization
 ******************************************************************************/
void Robot_ESP32_SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	

	Robot_ESP32_SPI2_DMA_Init();
	SPI_I2S_DeInit(SPI2); 	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);
/*ESP32 RST GPIOA.9 Output--------------------------------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ESP32_OFF(); 
/*ESP32 INT GPIOB.5 Output- Interupt ACT is Low----------------------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	ESP32_INT_DIS(); 
/*ESP32 CS GPIOA.8 Input----------------------------------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  				// Active is Low,Normal is High
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;																		
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	
	NVIC_SetPriority(EXTI9_5_IRQn,6);							//IRQ6									
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	
/*ESP32 ,GPIOB.14 MISO Out, ---------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15 |GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOB,GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15); 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	//����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;						//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;					//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ2  36/16=2.25MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 0;							//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure);  								//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

/*SPI2 Interrupt------------------------------------------------------------*/													
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);				 
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE,   ENABLE);	
	//SPI_I2S_ClearFlag(SPI2, SPI_I2S_FLAG_RXNE);
	//NVIC_SetPriority(SPI2_IRQn,2);									
	//NVIC_EnableIRQ(SPI2_IRQn);									
  	               
/*Enable SPI2 and DMA--------------------------------------------------*/
	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);  
	SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Rx,ENABLE); 							
/*DMA1 Channel5 and 4 Interrupt------------------------------------------------------------*/
	NVIC_SetPriority(DMA1_Channel5_IRQn,3);							// DMA1 CH5 TX--- IRQ3
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);								// Enable DMA1 CH5
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);					// Transfer finish interrupt	

	NVIC_SetPriority(DMA1_Channel4_IRQn,2);							// DMA1 CH5 RX--- IRQ2
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);								// Enable DMA1 CH4
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);					// Transfer finish interrupt	
}

/*******************************************************************************
* Routine     : Robot_ESP32_SPI2_DMA_Init
 * Function    : ESP32 spi2 DMA RX initial
 ******************************************************************************/
void Robot_ESP32_SPI2_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						//DMA1 clock enable
	
//SPI2 DMA1 TX---------------------------------------------------------//

    	DMA_DeInit(DMA1_Channel5);  												//DMA1  --TX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);  				//DMA1  --TX
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Esp32_tx_buf;  					//DMA1�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  						//���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = ESP_TX_SIZE;  							//DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  				//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  		//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  							//��������������ģʽ single TX
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 							//DMAͨ�� xӵ�������ȼ� 1
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  								//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);  		

//SPI2 DMA1 RX---------------------------------------------------------//		
    	DMA_DeInit(DMA1_Channel4);  												//DMA1  --RX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&SPI2->DR);  				//DMA1  --RX
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Esp32_rx_buf;  				//DMA1�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						//���ݴ��䷽�򣬴������ȡ���͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = ESP_RX_SIZE;  							//DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  				//�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  		//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  							//��������������ģʽ ccv RX
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 							//DMAͨ�� xӵ�������ȼ� 2
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  								//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  
}

/*****************************************************************
 * Routine     : RX_Check_Right
 * Function    : CheckSum for the ESP32 RX data
 *****************************************************************/
u8 RX_Check_Right(u16 Size, u8 *Adrr)
{
	u8 sum=0;
	u8 *sptr;
	u16 cnt16 = 0;
	
	sptr = Adrr;
	
	for(cnt16 = 0;cnt16<(Size-2);cnt16++)
	{
		sum += *sptr;
		sptr++;
	}
	if (sum == *sptr)    return 1;
	else	             return 0;
}

/*******************************************************************************
 * Routine     : RXD_Data_ESP32
 * Function    : Unpick the Frames of Reception Data from ESP32
 ******************************************************************************/
static void RXD_Data_ESP32(void)
{	
	tGear_TX_Frame spi_motor;

	if(Esp32_rx_buf[1] == 0x01)
	{
		spi_motor.Id = Esp32_rx_buf[2];
		spi_motor.Command = ACTION_TIME_COMMAND;
		spi_motor.Sel_uni_comb = TX_UNI;
		spi_motor.TX_done = 0;
		spi_motor.Action_times= Esp32_rx_buf[4];
		spi_motor.Angle = Esp32_rx_buf[3];

		Robot_Gear_Data_Input(spi_motor);
	}
}

/***************************************************************************
 * Routine     : TXD_Data_ESP32
 * Function    : Packge the Frames of Transmission Data to the ESP32
 ***************************************************************************/
static void TXD_Data_ESP32(void)
{
//   	u16 tmp16;
//    	u32 tmp32;

	Esp32_tx_buf[0] = 0x41;
	Esp32_tx_buf[1] = 0x41;
	
	DMA_Cmd(DMA1_Channel5, DISABLE); 
	DMA_SetCurrDataCounter(DMA1_Channel5,ESP_TX_SIZE);	
	DMA_Cmd(DMA1_Channel5, ENABLE ); 
} 

/*******************************************************************************
 * Routine     : Robot_ESP32_Commu_Main
 * Function    : Robot ESP32 Communicaton handle Main Loop 
 ******************************************************************************/
void Robot_ESP32_Commu_Main(void)
{
	u8 state;

	if(TmsFlag_ESPCom_handle)
	{
		TmsFlag_ESPCom_handle = 0;

		state = Robot_State_Out(ROBOT_SystemState);
		if(state == ROBOT_STATE_RUN)
		{
// test
			//RXD_Data_ESP32();
// end		 
			//ESP32_ON();	

			if(RXD_Done_Flag)
			{		
				if( RX_Check_Right(ESP_RX_SIZE,Esp32_rx_buf))
				{
					RXD_Data_ESP32();    
					//TXD_Data_ESP32();		
				}
				RXD_Done_Flag = 0;		// RX data handle is finish , active SPI and DMA
			}
		}
	}
}

  
/*******************************************************************************
 * Routine     : DMA1_Channel5_IRQHandler
 * Function    : DMA1 CH5-SPI2 TX IRQ Handle
 ******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5))
	{
		DMA_ClearITPendingBit(DMA1_FLAG_TC5);
		while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == 0);	
		SPI_Cmd(SPI2, DISABLE); 
	}
}

/*******************************************************************************
 * Routine     : DMA1_Channel4_IRQHandler
 * Function    : DMA1 CH4-SPI2 RX IRQ Handle
 ******************************************************* ***********************/
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4))
	{	
		RXD_Done_Flag = 1;
		SPI_Cmd(SPI2, DISABLE); 
		DMA_ClearITPendingBit(DMA1_FLAG_TC4);
	}
}

/******************************************************************************
 * Routine     : ESP32_IRQHandler
 * Function    : SPI2 CS Start RX Interrupt Handle,CS is Low active SPI2 and DMA
 ******************************************************************************/
void ESP32_IRQHandler(void)
{
	if(!RXD_Done_Flag)			// RX data is handle?
	{
		memset(Esp32_rx_buf,0,sizeof(Esp32_rx_buf));
		
		DMA_Cmd(DMA1_Channel4, DISABLE); 
		DMA_SetCurrDataCounter(DMA1_Channel4,ESP_RX_SIZE);	
		DMA_Cmd(DMA1_Channel4, ENABLE ); 
		SPI_Cmd(SPI2, ENABLE);
	}
}

/******************************************************************************
 * Routine     : EXTI9_5_IRQHandler
 * Function    : SPI2 CS Start RX Interrupt Handle
 ******************************************************************************/
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetFlagStatus(EXTI_Line8) != RESET)		// ESP32 handle
	{
		ESP32_IRQHandler();
		EXTI_ClearITPendingBit(EXTI_Line8);  							 						
	}
	
	//if(EXTI_GetFlagStatus(EXTI_Line6) != RESET)		// Voice handle
	//{		
	//	Sound_IRQHandler();
	//	EXTI_ClearITPendingBit(EXTI_Line6);  							 						
	//}
}

/*******************************************************************************
 * Routine     : SPI2_IRQHandler
 * Function    : 
 ******************************************************************************/
void SPI2_IRQHandler(void)
{
	u8 data;
	
	if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE)!=RESET)								
	{
		data = SPI_I2S_ReceiveData(SPI2);
		SPI_I2S_ClearITPendingBit(SPI2,SPI_I2S_FLAG_RXNE);			
		SPI_Cmd(SPI2, DISABLE); 
	}
}

/*******************************************************************************
 * Routine     : Robot_ESP32Com_TimeT1ms
 * Function    : ESP32 comunication 1ms interrupt
 ******************************************************************************/
void Robot_ESP32Com_TimeT1ms(void)
{
    static u8 T1msCnt=0;

/*20ms-------------------------------------------------------------------*/
	if(++T1msCnt >= 20)
	{
		T1msCnt = 0;

		TmsFlag_ESPCom_handle = 1;
	}
}


