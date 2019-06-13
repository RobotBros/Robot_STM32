/*******************************************************************************
 * FileName：Robot_Gear.c
 * Function: Robot Gear drive function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	    
#include "Robot_Gear.h"

/*Array Size define-------------------------------------------------------------*/
#define GEAR_TXD_SZIE     	10 				// Gear1 to Gear16 Command Action = 10*16 = 160 Bytes
#define GEAR_RXD_SZIE     	10
/*static variable define----------------------------------------------------------*/ 
static u8 Gear_tx_buf[GEAR_TXD_SZIE] = {0}; 
static u8 Gear_rx_buf[GEAR_RXD_SZIE] = {0}; 
static tGear_RX_Frame Gear_rx_frame;
extern tGear_TX_Frame Gear_tx_frame;
static u8 T2msFlag_Gear_Action = 0;			// Gear action time gap fLag
static u8 Motor_ReadAngle_Init_Done_Flag = 0;	
//static u8 Motor_Init_Done_Flag = 0;
static u8 Gear_TX_Frame_Done_Flag = 1;		// DMA TX is done flag
static u8 Gear_RX_Frame_Done_Flag = 0;		// DMA RX id done flag

static u16 *AngleReg_Addr[16];

/******************************************************************************
 * Routine     : Robot_Steering_Gear_Init
 * Function    : Usart2 For Steering Gear Initialization
 ******************************************************************************/
void Robot_Steering_Gear_Init(void)
{
/*Usart3  DIR I/O-----------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);			// Enable GPIOB clk
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2 ;							// PB2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					// Push pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;					// IO speed is 2MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);								// Write GPIO config
	GEAR_DIR_TX();													// Gear DIR TX mode
 /*Usart3 Initialization-------------------------------------------------*/
	Robot_Uart3_Init(115200);	 									// Gear Lower left interface
 /*Gear TX buffer Initialization-------------------------------------------------*/
	Robot_Gear_Var_Init();
}

/******************************************************************************
 * Routine     : Robot_Uart3_Init
 * Function    : Usart3 For Steering Gear Initialization
 ******************************************************************************/
void Robot_Uart3_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
/*Initial USART3 DMA Funciton----------------------------------------------------*/
	Robot_Usart3_DMA_Init();
/*Initial USART Clock-----------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);							// Enable GPIOB  				
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);							// Enable Usart3 clk
	USART_DeInit(USART3); 															// Reset Usart3 register
	
/*USART3 I/O TX GPIOB.10-----------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;															
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
/*USART3 I/O RX GPIOB.11-----------------------------------------------------*/ 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;								
  	GPIO_Init(GPIOB, &GPIO_InitStructure);  

/*USART3 Initialization---------------------------------------------------------*/
	USART_InitStructure.USART_BaudRate = bound;										
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// Data length is 8 bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// One stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;									// No check bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// No hardware ctrl bit
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// RX and TX mode
  	USART_Init(USART3, &USART_InitStructure); 											
																																//如果使能了接收  
/*USART3 Interrupt------------------------------------------------------------*/
  	//NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;								
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;									
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									
	//NVIC_Init(&NVIC_InitStructure);														
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);				 
	//USART_ITConfig(USART3, USART_IT_TXE,   ENABLE);	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	NVIC_SetPriority(USART3_IRQn,5);									// RX----IRQ5					
	NVIC_EnableIRQ(USART3_IRQn);									
  	               
/*Enable USART3 and DMA--------------------------------------------------*/
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE); 				//使能USART3的 TX DMA传输    
	//USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 				//使能USART3的 RX DMA传输  
    USART_Cmd(USART3, ENABLE); 								   	

/*DMA1 Channel2 and 3 Interrupt------------------------------------------------------------*/
	NVIC_SetPriority(DMA1_Channel2_IRQn,4);							// DMA1 CH2 TX--- IRQ4
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);								// Enable DMA1 CH2
	DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);					// Transfer finish interrupt	
}

/******************************************************************************
 * Routine     : Robot_Usart3_DMA_Init
 * Function    : Usart3 DMA TX/RX initial
 ******************************************************************************/
void Robot_Usart3_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						//DMA1 clock enable
	
//USART3 DMA1 TX---------------------------------------------------------//	
    DMA_DeInit(DMA1_Channel2);  												//DMA1 CH2 --TX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);  				//DMA1 USART3 --TX
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Gear_tx_buf;  				//DMA1内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  						//数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = GEAR_TXD_SZIE;  							//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  				//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  							//工作在正常缓存模式 single TX
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有中优先级 1
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  								//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);  		
/*
//USART3 DMA1 RX---------------------------------------------------------//		
    	DMA_DeInit(DMA1_Channel3);  												//DMA1 CH3 --RX
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);  				//DMA1 USART3 --RX
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Gear_rx_buf;  				//DMA1内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						//数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = GEAR_RXD_SZIE;  							//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  				//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							//工作在正常缓存模式 ccv RX
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 							//DMA通道 x拥有中优先级 2
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  								//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);  
*/
}

/*******************************************************************************
 * Routine     : Robot_Gear_Var_Init
 * Function    : Robot  Gear Variable Initialization 
 ******************************************************************************/
void Robot_Gear_Var_Init(void)
{
	Gear_tx_frame.Id = 1;
	Gear_tx_frame.Command = ACTION_IDEL;
	Gear_tx_frame.Sel_uni_comb = TX_UNI;

	Gear_rx_frame.Id = 1;
	Gear_rx_frame.Command = ACTION_IDEL;
	Gear_rx_frame.RX_done_flag = 1;
//test
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = 1;							
	Gear_tx_buf[3] = RD_ANGLE_COMMAND;		
	Gear_tx_buf[4] = 0;							
	Gear_tx_buf[5] = 0;							
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;						//ED
/*Motor angle ver----------------------------------------*/
	AngleReg_Addr[0] = (u16*)&Gear_rx_frame.Id1_Angle;
	AngleReg_Addr[1] = (u16*)&Gear_rx_frame.Id2_Angle;
	AngleReg_Addr[2] = (u16*)&Gear_rx_frame.Id3_Angle;
	AngleReg_Addr[3] = (u16*)&Gear_rx_frame.Id4_Angle;
	AngleReg_Addr[4] = (u16*)&Gear_rx_frame.Id5_Angle;
	AngleReg_Addr[5] = (u16*)&Gear_rx_frame.Id6_Angle;
	AngleReg_Addr[6] = (u16*)&Gear_rx_frame.Id7_Angle;
	AngleReg_Addr[7] = (u16*)&Gear_rx_frame.Id8_Angle;
	AngleReg_Addr[8] = (u16*)&Gear_rx_frame.Id9_Angle;
	AngleReg_Addr[9] = (u16*)&Gear_rx_frame.Id10_Angle;
	AngleReg_Addr[10] = (u16*)&Gear_rx_frame.Id11_Angle;
	AngleReg_Addr[11] = (u16*)&Gear_rx_frame.Id12_Angle;
	AngleReg_Addr[12] = (u16*)&Gear_rx_frame.Id13_Angle;
	AngleReg_Addr[13] = (u16*)&Gear_rx_frame.Id14_Angle;
	AngleReg_Addr[14] = (u16*)&Gear_rx_frame.Id15_Angle;
	AngleReg_Addr[15] = (u16*)&Gear_rx_frame.Id16_Angle;
}

/*******************************************************************************
 * Routine   : Robot_Gear_Angle_Out
 * Function  : Read Robot Fault flag 
 * Input      : regist --motor ID :MOTORx
 * Output	   : Erro return -1 , else return motor ID angle
 ******************************************************************************/
s16 Robot_Gear_Angle_Out(u8 regist)
{  
	if(Gear_rx_frame.RX_done_flag)
	{
		Gear_rx_frame.RX_done_flag = 0;
		
		if(regist < 17 ||regist > 0)	return (*AngleReg_Addr[regist]);
		else 	return(-1);
	}	
	else		return(-1);
}

/*******************************************************************************
 * Routine   : Robot_Gear_Angle_Cla
 * Function  : Caculate motor angle
 ******************************************************************************/
void Robot_Gear_Angle_Cla(u8 dir)
{  
	if(dir == ADD)
	{
		if(Gear_tx_frame.Sel_uni_comb == TX_UNI)
		{
			switch(Gear_tx_frame.Id)
			{
				case 1: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id1_AngleOffset; break;
				case 2: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id2_AngleOffset; break;
				case 3: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id3_AngleOffset; break;
				case 4: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id4_AngleOffset; break;
				case 5: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id5_AngleOffset; break;
				case 6: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id6_AngleOffset; break;
				case 7: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id7_AngleOffset; break;
				case 8: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id8_AngleOffset; break;
				case 9: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id9_AngleOffset; break;
				case 10: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id10_AngleOffset; break;
				case 11: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id11_AngleOffset; break;
				case 12: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id12_AngleOffset; break;
				case 13: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id13_AngleOffset; break;
				case 14: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id14_AngleOffset; break;
				case 15: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id15_AngleOffset; break;
				case 16: Gear_tx_frame.Angle = Gear_tx_frame.Angle + Gear_rx_frame.Id16_AngleOffset; break;
				default:break;
			}
		}
	
		else if(Gear_tx_frame.Sel_uni_comb == TX_COMB)
		{
			Gear_tx_frame.Id1_Angle = Gear_tx_frame.Id1_Angle + Gear_rx_frame.Id1_AngleOffset;
			Gear_tx_frame.Id2_Angle = Gear_tx_frame.Id2_Angle + Gear_rx_frame.Id2_AngleOffset;
			Gear_tx_frame.Id3_Angle = Gear_tx_frame.Id3_Angle + Gear_rx_frame.Id3_AngleOffset;
			Gear_tx_frame.Id4_Angle = Gear_tx_frame.Id4_Angle + Gear_rx_frame.Id4_AngleOffset;
			Gear_tx_frame.Id5_Angle = Gear_tx_frame.Id5_Angle + Gear_rx_frame.Id5_AngleOffset;
			Gear_tx_frame.Id6_Angle = Gear_tx_frame.Id6_Angle + Gear_rx_frame.Id6_AngleOffset;
			Gear_tx_frame.Id7_Angle = Gear_tx_frame.Id7_Angle + Gear_rx_frame.Id7_AngleOffset;
			Gear_tx_frame.Id8_Angle = Gear_tx_frame.Id8_Angle + Gear_rx_frame.Id8_AngleOffset;
			Gear_tx_frame.Id9_Angle = Gear_tx_frame.Id9_Angle + Gear_rx_frame.Id9_AngleOffset;
			Gear_tx_frame.Id10_Angle = Gear_tx_frame.Id10_Angle + Gear_rx_frame.Id10_AngleOffset;
			Gear_tx_frame.Id11_Angle = Gear_tx_frame.Id11_Angle + Gear_rx_frame.Id11_AngleOffset;
			Gear_tx_frame.Id12_Angle = Gear_tx_frame.Id12_Angle + Gear_rx_frame.Id12_AngleOffset;
			Gear_tx_frame.Id13_Angle = Gear_tx_frame.Id13_Angle + Gear_rx_frame.Id13_AngleOffset;
			Gear_tx_frame.Id14_Angle = Gear_tx_frame.Id14_Angle + Gear_rx_frame.Id14_AngleOffset;
			Gear_tx_frame.Id15_Angle = Gear_tx_frame.Id15_Angle + Gear_rx_frame.Id15_AngleOffset;
			Gear_tx_frame.Id16_Angle = Gear_tx_frame.Id16_Angle + Gear_rx_frame.Id16_AngleOffset;
		}
	}

	if(dir == MINUS)
	{
		if(Gear_tx_frame.Sel_uni_comb == TX_UNI)
		{
			switch(Gear_tx_frame.Id)
			{
				case 1: Gear_tx_frame.Angle =  Gear_rx_frame.Id1_AngleOffset - Gear_tx_frame.Angle; break;
				case 2: Gear_tx_frame.Angle = Gear_rx_frame.Id2_AngleOffset - Gear_tx_frame.Angle; break;
				case 3: Gear_tx_frame.Angle = Gear_rx_frame.Id3_AngleOffset - Gear_tx_frame.Angle; break;
				case 4: Gear_tx_frame.Angle = Gear_rx_frame.Id4_AngleOffset - Gear_tx_frame.Angle; break;
				case 5: Gear_tx_frame.Angle = Gear_rx_frame.Id5_AngleOffset - Gear_tx_frame.Angle; break;
				case 6: Gear_tx_frame.Angle = Gear_rx_frame.Id6_AngleOffset - Gear_tx_frame.Angle; break;
				case 7: Gear_tx_frame.Angle = Gear_rx_frame.Id7_AngleOffset - Gear_tx_frame.Angle; break;
				case 8: Gear_tx_frame.Angle = Gear_rx_frame.Id8_AngleOffset - Gear_tx_frame.Angle; break;
				case 9: Gear_tx_frame.Angle = Gear_rx_frame.Id9_AngleOffset - Gear_tx_frame.Angle; break;
				case 10: Gear_tx_frame.Angle = Gear_rx_frame.Id10_AngleOffset - Gear_tx_frame.Angle; break;
				case 11: Gear_tx_frame.Angle = Gear_rx_frame.Id11_AngleOffset - Gear_tx_frame.Angle; break;
				case 12: Gear_tx_frame.Angle = Gear_rx_frame.Id12_AngleOffset - Gear_tx_frame.Angle; break;
				case 13: Gear_tx_frame.Angle = Gear_rx_frame.Id13_AngleOffset - Gear_tx_frame.Angle; break;
				case 14: Gear_tx_frame.Angle = Gear_rx_frame.Id14_AngleOffset - Gear_tx_frame.Angle; break;
				case 15: Gear_tx_frame.Angle = Gear_rx_frame.Id15_AngleOffset - Gear_tx_frame.Angle; break;
				case 16: Gear_tx_frame.Angle = Gear_rx_frame.Id16_AngleOffset - Gear_tx_frame.Angle; break;
				default:break;
			}
		}
	
		else if(Gear_tx_frame.Sel_uni_comb == TX_COMB)
		{
			Gear_tx_frame.Id1_Angle = Gear_rx_frame.Id1_AngleOffset - Gear_tx_frame.Id1_Angle;
			Gear_tx_frame.Id2_Angle = Gear_rx_frame.Id2_AngleOffset - Gear_tx_frame.Id2_Angle;
			Gear_tx_frame.Id3_Angle = Gear_rx_frame.Id3_AngleOffset - Gear_tx_frame.Id3_Angle;
			Gear_tx_frame.Id4_Angle = Gear_rx_frame.Id4_AngleOffset - Gear_tx_frame.Id4_Angle;
			Gear_tx_frame.Id5_Angle = Gear_rx_frame.Id5_AngleOffset - Gear_tx_frame.Id5_Angle;
			Gear_tx_frame.Id6_Angle = Gear_rx_frame.Id6_AngleOffset - Gear_tx_frame.Id6_Angle;
			Gear_tx_frame.Id7_Angle = Gear_rx_frame.Id7_AngleOffset - Gear_tx_frame.Id7_Angle;
			Gear_tx_frame.Id8_Angle = Gear_rx_frame.Id8_AngleOffset - Gear_tx_frame.Id8_Angle;
			Gear_tx_frame.Id9_Angle = Gear_rx_frame.Id9_AngleOffset - Gear_tx_frame.Id9_Angle;
			Gear_tx_frame.Id10_Angle = Gear_rx_frame.Id10_AngleOffset - Gear_tx_frame.Id10_Angle;
			Gear_tx_frame.Id11_Angle =Gear_rx_frame.Id11_AngleOffset - Gear_tx_frame.Id11_Angle;
			Gear_tx_frame.Id12_Angle = Gear_rx_frame.Id12_AngleOffset - Gear_tx_frame.Id12_Angle;
			Gear_tx_frame.Id13_Angle = Gear_rx_frame.Id13_AngleOffset - Gear_tx_frame.Id13_Angle;
			Gear_tx_frame.Id14_Angle = Gear_rx_frame.Id14_AngleOffset - Gear_tx_frame.Id14_Angle;
			Gear_tx_frame.Id15_Angle = Gear_rx_frame.Id15_AngleOffset - Gear_tx_frame.Id15_Angle;
			Gear_tx_frame.Id16_Angle = Gear_rx_frame.Id16_AngleOffset - Gear_tx_frame.Id16_Angle;
		}
	}	
}
	
/*******************************************************************************
 * Routine   : Robot_Gear_Data_Input
 * Function  : Read Robot data input comb
 ******************************************************************************/
void Robot_Gear_Data_Input(tGear_TX_Frame pmotor)
{  
	Gear_tx_frame.Id = pmotor.Id;
	Gear_tx_frame.Command = pmotor.Command;
	Gear_tx_frame.Led_flash = pmotor.Led_flash;
	Gear_tx_frame.Angle = pmotor.Angle;
	Gear_tx_frame.Action_times = pmotor.Action_times;
	Gear_tx_frame.TX_done = pmotor.TX_done;
	Gear_tx_frame.Sel_uni_comb = pmotor.Sel_uni_comb;
	
	Gear_tx_frame.Id1_Angle = pmotor.Id1_Angle + Gear_rx_frame.Id1_AngleOffset;
	Gear_tx_frame.Id2_Angle = pmotor.Id2_Angle + Gear_rx_frame.Id2_AngleOffset;
	Gear_tx_frame.Id3_Angle = pmotor.Id3_Angle + Gear_rx_frame.Id3_AngleOffset;
	Gear_tx_frame.Id4_Angle = pmotor.Id4_Angle + Gear_rx_frame.Id4_AngleOffset;
	Gear_tx_frame.Id5_Angle = pmotor.Id5_Angle + Gear_rx_frame.Id5_AngleOffset;
	Gear_tx_frame.Id6_Angle = pmotor.Id6_Angle + Gear_rx_frame.Id6_AngleOffset;
	Gear_tx_frame.Id7_Angle = pmotor.Id7_Angle + Gear_rx_frame.Id7_AngleOffset;
	Gear_tx_frame.Id8_Angle = pmotor.Id8_Angle + Gear_rx_frame.Id8_AngleOffset;
	Gear_tx_frame.Id9_Angle = pmotor.Id9_Angle + Gear_rx_frame.Id9_AngleOffset;
	Gear_tx_frame.Id10_Angle = pmotor.Id10_Angle + Gear_rx_frame.Id10_AngleOffset;
	Gear_tx_frame.Id11_Angle = pmotor.Id11_Angle + Gear_rx_frame.Id11_AngleOffset;
	Gear_tx_frame.Id12_Angle = pmotor.Id12_Angle + Gear_rx_frame.Id12_AngleOffset;
	Gear_tx_frame.Id13_Angle = pmotor.Id13_Angle + Gear_rx_frame.Id13_AngleOffset;
	Gear_tx_frame.Id14_Angle = pmotor.Id14_Angle + Gear_rx_frame.Id14_AngleOffset;
	Gear_tx_frame.Id15_Angle = pmotor.Id15_Angle + Gear_rx_frame.Id15_AngleOffset;
	Gear_tx_frame.Id16_Angle = pmotor.Id16_Angle + Gear_rx_frame.Id16_AngleOffset;

	Gear_tx_frame.Id1_Times = pmotor.Id1_Times;
	Gear_tx_frame.Id2_Times = pmotor.Id2_Times;
	Gear_tx_frame.Id3_Times = pmotor.Id3_Times;
	Gear_tx_frame.Id4_Times = pmotor.Id4_Times;
	Gear_tx_frame.Id5_Times = pmotor.Id5_Times;
	Gear_tx_frame.Id6_Times = pmotor.Id6_Times;
	Gear_tx_frame.Id7_Times = pmotor.Id7_Times;
	Gear_tx_frame.Id8_Times = pmotor.Id8_Times;
	Gear_tx_frame.Id9_Times = pmotor.Id9_Times;
	Gear_tx_frame.Id10_Times = pmotor.Id10_Times;
	Gear_tx_frame.Id11_Times = pmotor.Id11_Times;
	Gear_tx_frame.Id12_Times = pmotor.Id12_Times;
	Gear_tx_frame.Id13_Times = pmotor.Id13_Times;
	Gear_tx_frame.Id14_Times = pmotor.Id14_Times;
	Gear_tx_frame.Id15_Times = pmotor.Id15_Times;
	Gear_tx_frame.Id16_Times = pmotor.Id16_Times;
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_UniTX_Frame
 * Function    : Robot Gear Normal TX Frame handle uni ID motor
 ******************************************************************************/
void Robot_Gear_Nor_UniTX_Frame(void)
{						
	switch(Gear_tx_frame.Command)
	{
		case RD_ANGLE_COMMAND:
				Robot_Gear_Nor_ReadAngle_TX(Gear_tx_frame.Id);
				GEAR_DIR_TX();
				DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART3 TX DMA1 CH2 所指示的通道      
 				DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 				DMA_Cmd(DMA1_Channel2, ENABLE);  
				USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
				break;
		case ACTION_TIME_COMMAND:
				Robot_Gear_Nor_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Angle,Gear_tx_frame.Action_times);
				GEAR_DIR_TX();
				DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART3 TX DMA1 CH2 所指示的通道      
 				DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 				DMA_Cmd(DMA1_Channel2, ENABLE);  
				USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
				break;
		case LED_COMMAND:
				Robot_Gear_Nor_LED_TX(Gear_tx_frame.Id,Gear_tx_frame.Led_flash);
				GEAR_DIR_TX();
				DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART3 TX DMA1 CH2 所指示的通道      
 				DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 				DMA_Cmd(DMA1_Channel2, ENABLE);  
				USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
				break;
		case MODIFY_ID_COMMAND:
				Robot_Gear_Nor_ID_TX(Gear_tx_frame.Id);
				GEAR_DIR_TX();
				DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART3 TX DMA1 CH2 所指示的通道      
 				DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 				DMA_Cmd(DMA1_Channel2, ENABLE);  
				USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
				break;
		case ANGLE_ADJ_COMMAND:break;
		case RD_ADJ_COMMAND:break;
		default:break;			
	}	
}

/******************************************************************************
 * Routine     : Robot_Gear_Check_sum
 * Function    : Gear frame checksum
 * check sum = ID+Command+par1+par2+par3+par4
 * buf[2]~buf[7]
 ******************************************************************************/
u8 Robot_Gear_Check_sum(u8 *buf, u8 start_byte_no, u8 end_byte_no)		
{
	u8 check_sum = 0;
	u8 cnt = 0;

	for(cnt=start_byte_no;cnt<end_byte_no;cnt++)
		{check_sum += buf[cnt];}
	return check_sum;
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_LED_TX
 * Function    : Robot Gear Normal LED TX handle 
 ******************************************************************************/
void Robot_Gear_Nor_LED_TX(u8 id,u8 on_off)
{
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = id;								// ID 
	Gear_tx_buf[3] = LED_COMMAND;					
	Gear_tx_buf[4] = on_off;							// 1:ON 0:OFF
	Gear_tx_buf[5] = 0;							
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;							//ED
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_ID_TX
 * Function    : Robot Gear Normal Modify TX handle 
 ******************************************************************************/
void Robot_Gear_Nor_ID_TX(u8 id)
{
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = 0;								
	Gear_tx_buf[3] = MODIFY_ID_COMMAND;					
	Gear_tx_buf[4] = 0;							
	Gear_tx_buf[5] = id;							
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;							//ED
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_Action_TX
 * Function    : Robot Gear Normal Action TX handle 
 ******************************************************************************/
void Robot_Gear_Nor_Action_TX(u8 id,u8 angle,u8 time)
{
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = id;								// ID 
	Gear_tx_buf[3] = ACTION_TIME_COMMAND;			// Action Command
	Gear_tx_buf[4] = angle;							// Angle
	Gear_tx_buf[5] = time;							// Action time
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;						//ED
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_ReadAngle_TX
 * Function    : Robot Gear Normal Read Motor Angle TX handle --single ID motor
 ******************************************************************************/
void Robot_Gear_Nor_ReadAngle_TX(u8 id)
{			
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = id;
	Gear_tx_buf[3] = RD_ANGLE_COMMAND;		
	Gear_tx_buf[4] = 0;							
	Gear_tx_buf[5] = 0;							
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
} 

//--------------------------------------Initial action---------------------------------------------//
/******************************************************************************
 * Routine     : Robot_Gear_Action_TX
 * Function    : Robot Gear Initial Action TX handle 
 ******************************************************************************/
void Robot_Gear_Action_TX(u8 id,u8 angle,u8 time)
{
	Gear_rx_frame.Command = ACTION_TIME_COMMAND;

	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = id;								// ID 
	Gear_tx_buf[3] = ACTION_TIME_COMMAND;			// Action Command
	Gear_tx_buf[4] = angle;							// Angle
	Gear_tx_buf[5] = time;							// Action time
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;						//ED

	GEAR_DIR_TX();
	DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART2 TX DMA1 CH7 所指示的通道      
 	DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA1_Channel2, ENABLE);  
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
}

/******************************************************************************
 * Routine     : Robot_Gear_ReadAngle_TX
 * Function    : Robot Gear Read Motor Angle TX handle --16 ID motor
 ******************************************************************************/
void Robot_Gear_ReadAngle_TX(void)
{
	static u8 id8 = 1;

	if(Gear_rx_frame.RX_done_flag)
	{
		Gear_rx_frame.RX_done_flag = 0;
		if(id8>16) id8=1;					
		Gear_tx_buf[2] = id8;
		Gear_tx_buf[3] = RD_ANGLE_COMMAND;		
		Gear_tx_buf[4] = 0;							
		Gear_tx_buf[5] = 0;							
		Gear_tx_buf[6] = 0;
		Gear_tx_buf[7] = 0;
		Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
		Gear_rx_frame.Id = id8;
		id8++;
	}
	
	DMA_Cmd(DMA1_Channel2, DISABLE );  						//关闭USART3 TX DMA1 CH2 所指示的通道      
 	DMA_SetCurrDataCounter(DMA1_Channel2,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA1_Channel2, ENABLE);  						//使能USART3 TX DMA1 CH2 所指示的通道 
 	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
} 

/******************************************************************************
 * Routine     : Robot_Gear_Init_TX_Handle
 * Function    : Robot Gear initial motor TX handle
 ******************************************************************************/
void Robot_Gear_Init_TX_Handle(u8 timer)
{
	static u8 id8_tmp = 1;
	
	GEAR_DIR_TX();	

	if(!Motor_ReadAngle_Init_Done_Flag)				// Initial -->read angle hole ID motor
	{
		Robot_Gear_ReadAngle_TX();				
	}
	else												// Initial--> read angle feed back to motor action
	{
		if(id8_tmp > 16) id8_tmp = 1;
		switch(id8_tmp)
		{
			case 1:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id1_AngleOffset,timer);break;			
			case 2:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id2_AngleOffset,timer);break;			
			case 3:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id3_AngleOffset,timer);break;			
			case 4:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id4_AngleOffset,timer);break;			
			case 5:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id5_AngleOffset,timer);break;			
			case 6:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id6_AngleOffset,timer);break;			
			case 7:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id7_AngleOffset,timer);break;			
			case 8:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id8_AngleOffset,timer);break;			
			case 9:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id9_AngleOffset,timer);break;			
			case 10:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id10_AngleOffset,timer);break;		
			case 11:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id11_AngleOffset,timer);break;	
			case 12:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id12_AngleOffset,timer);break;
			case 13:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id13_AngleOffset,timer);break;
			case 14:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id14_AngleOffset,timer);break;
			case 15:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id15_AngleOffset,timer);break;
			case 16:Robot_Gear_Action_TX(id8_tmp,Gear_rx_frame.Id16_AngleOffset,timer);
				      Robot_Command_Motor_Enable(ON);				
				      //Gear_TX_Frame_Done_Flag = 1;
				      break;
			default:break;
		}	
		id8_tmp++;
	}
}

/******************************************************************************
 * Routine     : Robot_Gear_Nor_CombTX_Frame
 * Function    : Robot Gear Normal TX Frame handle 16 ID motor
 ******************************************************************************/
void Robot_Gear_Nor_CombTX_Frame(void)
{
	static u8 id8_tmp = 1;
	
	GEAR_DIR_TX();	

	if(id8_tmp > 16) id8_tmp = 1;
	Gear_tx_frame.Id= id8_tmp;
	switch(Gear_tx_frame.Id)
	{
		case 1:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id1_Times);break;			
		case 2:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id2_Times);break;			
		case 3:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id3_Times);break;			
		case 4:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id4_Times);break;			
		case 5:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id5_Times);break;			
		case 6:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id6_Times);break;			
		case 7:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id7_Times);break;			
		case 8:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id8_Times);break;			
		case 9:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id9_Times);break;			
		case 10:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id10_Times);break;		
		case 11:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id11_Times);break;	
		case 12:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id12_Times);break;
		case 13:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id13_Times);break;
		case 14:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id14_Times);break;
		case 15:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id15_Times);break;
		case 16:Robot_Gear_Action_TX(Gear_tx_frame.Id,Gear_tx_frame.Id1_Angle,Gear_tx_frame.Id16_Times);break;
		default:break;
	}	
	id8_tmp++;
}

/*******************************************************************************
 * Routine     : Gear_RX_Check_Right
 * Function    : Gear RX data CheckSum OK
 ******************************************************************************/
 /* Routine     : Gear_Tx_Frame
 * Function    : Gear Tx frame buffer
 *TX frame:0xFA+0xAF+ID+command+parm1H+parm1L+parm2H+parm2L+checksum+0xED		Lenth 10 bytes
 *TX checksum =  ID+command+parm1H+parm1L+parm2H+parm2L							Lenth  1 byte
 *RX frame:0xFA+0xAF+ID+status+parm1H+parm1L+parm2H+parm2L+checksum+0xED			Lenth 10 bytes
 *TX checksum =  ID+status+parm1H+parm1L+parm2H+parm2L								Lenth  1 byte
 ******************************************************************************/
u8 Gear_RX_Check_Right(u8 *Adrr,u8 start_byte_no, u8 end_byte_no)
{
	u8 check_sum=0;
	u8 cnt8=0; 
 
	for(cnt8=start_byte_no;cnt8<end_byte_no;cnt8++)
	{check_sum += Adrr[cnt8];}
	
	if(Adrr[8] == check_sum) return 1;
	else return 0;
}

/******************************************************************************
 * Routine     : Robot_Gear_Init_RX_Handle
 * Function    : Handle Motor Initial action RX data---read 16 ID motor initial angle
 ******************************************************************************/
void Robot_Gear_Init_RX_Handle(void)
{
	u8 flag8 = 0;

	flag8 = Gear_RX_Check_Right(Gear_rx_buf,ID1_CHECK_START,ID1_CHECK_END);
	
	if(flag8) 
	{
		Gear_rx_frame.RX_done_flag = 1;
		switch(Gear_tx_buf[2])
		{
			case 1:Gear_rx_frame.Id1_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 2:Gear_rx_frame.Id2_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 3:Gear_rx_frame.Id3_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 4:Gear_rx_frame.Id4_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 5:Gear_rx_frame.Id5_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 6:Gear_rx_frame.Id6_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 7:Gear_rx_frame.Id7_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 8:Gear_rx_frame.Id8_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 9:Gear_rx_frame.Id9_AngleOffset = Gear_rx_buf[7];break;			//GearX angle high byte
			case 10:Gear_rx_frame.Id10_AngleOffset = Gear_rx_buf[7];break;		//GearX angle high byte
			case 11:Gear_rx_frame.Id11_AngleOffset = Gear_rx_buf[7];break;	
			case 12:Gear_rx_frame.Id12_AngleOffset = Gear_rx_buf[7];break;
			case 13:Gear_rx_frame.Id13_AngleOffset = Gear_rx_buf[7];break;
			case 14:Gear_rx_frame.Id14_AngleOffset = Gear_rx_buf[7];break;
			case 15:Gear_rx_frame.Id15_AngleOffset = Gear_rx_buf[7];break;
			case 16:Gear_rx_frame.Id16_AngleOffset = Gear_rx_buf[7];Motor_ReadAngle_Init_Done_Flag=1;break;
			default:break;
		}
	}
	else Gear_rx_frame.RX_done_flag = 0;
}

/******************************************************************************
 * Routine     : Robot_Gear_Normal_RX_Handle
 * Function    : Handle Motor Normal action RX data---read 16 ID motor command
 ******************************************************************************/
void Robot_Gear_Normal_RX_Handle(void)
{
	u8 flag8 = 0;

	flag8 = Gear_RX_Check_Right(Gear_rx_buf,ID1_CHECK_START,ID1_CHECK_END);
	
	if(flag8) 
	{
		switch(Gear_tx_buf[2])
		{
			case 1:Gear_rx_frame.Id1_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 2:Gear_rx_frame.Id2_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 3:Gear_rx_frame.Id3_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 4:Gear_rx_frame.Id4_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 5:Gear_rx_frame.Id5_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 6:Gear_rx_frame.Id6_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 7:Gear_rx_frame.Id7_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 8:Gear_rx_frame.Id8_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 9:Gear_rx_frame.Id9_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;			//GearX angle high byte
			case 10:Gear_rx_frame.Id10_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;		//GearX angle high byte
			case 11:Gear_rx_frame.Id11_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;	
			case 12:Gear_rx_frame.Id12_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;
			case 13:Gear_rx_frame.Id13_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;
			case 14:Gear_rx_frame.Id14_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;
			case 15:Gear_rx_frame.Id15_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;
			case 16:Gear_rx_frame.Id16_AngleOffset = Gear_rx_buf[7];Gear_rx_frame.RX_done_flag = 1;break;
			default:break;
		}
	}
	else Gear_rx_frame.RX_done_flag = 0;
}

/******************************************************************************
 * Routine     : Robot_Gear_TX_Test
 * Function    : Handle tx data
 ******************************************************************************/
void Robot_Gear_TX_Test(void)
{
	tGear_TX_Frame test_motor;
//Modify ID
/*
	test_motor.Id = 13;							// Modify ID NO.
	test_motor.Command = MODIFY_ID_COMMAND;
	test_motor.Sel_uni_comb = TX_UNI;
	test_motor.TX_done = 0;
*/
//Test angle
/*
	test_motor.Id = 13;
	test_motor.Command = ACTION_TIME_COMMAND;
	test_motor.Sel_uni_comb = TX_UNI;
	test_motor.TX_done = 0;
	test_motor.Action_times= 50;
	test_motor.Angle = 40;
*/
//Read angle
	test_motor.Id = 13;							// Modify ID NO.
	test_motor.Command = RD_ANGLE_COMMAND;
	test_motor.Sel_uni_comb = TX_UNI;
	test_motor.TX_done = 0;
	
	Robot_Gear_Data_Input(test_motor);
	Robot_Gear_Nor_UniTX_Frame();
	
	delay_ms(100);
}

/*******************************************************************************
 * Routine     : Robot_Steering_Gear_Main
 * Function    : Robot Steering Gear Main Loop 
 ******************************************************************************/
void Robot_Steering_Gear_Main(void)
{	
	u8 state;
	//tGear_TX_Frame test;
	
	if(T2msFlag_Gear_Action)
	{
		T2msFlag_Gear_Action = 0;

		state = Robot_State_Out(ROBOT_SystemState);
/*----------Robot IDLE state--------------------------------------------------------------------------*/
		if(state == ROBOT_STATE_IDLE)
		{
			if(Gear_TX_Frame_Done_Flag)						// TX handle
			{
				Gear_TX_Frame_Done_Flag = 0;
				Robot_Gear_Init_TX_Handle(50);				// Motor initial action 
			}
			
			if(Gear_RX_Frame_Done_Flag)						// RX handle
			{
				Gear_RX_Frame_Done_Flag = 0;
				Gear_TX_Frame_Done_Flag = 1;
				USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);	// Close RX IRQ
				Robot_Gear_Init_RX_Handle();
			}
		}
/*----------Robot run state--------------------------------------------------------------------------*/		
		if(state == ROBOT_STATE_RUN)
		{
			//if(Gear_TX_Frame_Done_Flag)						// TX handle
			//{
			//	Gear_TX_Frame_Done_Flag = 0;
				
				if(!Gear_tx_frame.TX_done)
				{
					if(Gear_tx_frame.Sel_uni_comb==TX_UNI)		// TX uni ID motor
					{
						Robot_Gear_Nor_UniTX_Frame();
					}
					else
					{
						Robot_Gear_Nor_CombTX_Frame();
					}
				}
			//}	
			
			if(Gear_RX_Frame_Done_Flag)						// RX handle
			{
				Gear_RX_Frame_Done_Flag = 0;
				//Gear_TX_Frame_Done_Flag = 1;
				USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);	// Close RX IRQ
 				Robot_Gear_Normal_RX_Handle();	
			}
		}
/*----------Robot debug state--------------------------------------------------------------------------*/		
		else if(state == ROBOT_STATE_DEBUG)
		{
			// Debug mode action
		}
	}
}

/*******************************************************************************
 * Routine     : UartGear_RXD_IRQ
 * Function    : USIC3 RXD interrupt for Gear
 ******************************************************************************/
static void UartGear_RXD_IRQ(void)
{
 	u8  RecvData=0;
    	static u16 RecvCnt=0;

	RecvData = USART_ReceiveData(USART3);
	
	if(RecvCnt==0)
	{
		if(RecvData==0xFA)
		{
			Gear_rx_buf[0] = RecvData;
			RecvCnt++;
		}
	}
	else if(RecvCnt==1)
	{
		if(RecvData==0xAF)
		{
			Gear_rx_buf[1] = RecvData;
			RecvCnt++;
		}
		else  RecvCnt = 0;
	}
	else
	{
		Gear_rx_buf[RecvCnt] = RecvData;
		RecvCnt++;
		if(RecvCnt>=GEAR_RXD_SZIE)
		{
			Gear_RX_Frame_Done_Flag = 1;
			RecvCnt=0;
			//USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		}
	}	
}

/*******************************************************************************
 * Routine     : Robot_Steering_Gear_TimeT1ms
 * Function    : Robot Steering Gear timebase(2ms) Single Motor handle
 ******************************************************************************/
void Robot_Steering_Gear_TimeT1ms(void)
{
    static u16 T1msCnt=0;
    static u16 T200msTimeoutCnt=0;

/*2ms----------------------------------------------------------------------*/
	if(++T1msCnt>=2)
	{
		T1msCnt = 0;
		T2msFlag_Gear_Action = 1;
	}
/*200ms TX time out handle----------------------------------------------------*/
	if(Gear_TX_Frame_Done_Flag ==0)
	{
		if(++T200msTimeoutCnt>=200)
		{
			T200msTimeoutCnt = 0;
			Gear_TX_Frame_Done_Flag = 1;
		}
	}
	else T200msTimeoutCnt = 0;
}

/*******************************************************************************
 * Routine     : DMA1_Channel2_IRQHandler
 * Function    : DMA1 CH2-USART3 TX IRQ Handle
 ******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC2))
	{
		DMA_ClearITPendingBit(DMA1_FLAG_TC2);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC) == 0);	
		//Gear_TX_Frame_Done_Flag = 1;
		if(Gear_tx_frame.Sel_uni_comb == TX_UNI) 	{Gear_tx_frame.TX_done = 1;}
		if((Gear_tx_frame.Id == 16) && (Gear_tx_frame.Sel_uni_comb == TX_COMB)) {Gear_tx_frame.TX_done = 1;}
		Gear_TX_Frame_Done_Flag = 1;											// TX DMA Transfer is Done
		GEAR_DIR_RX();
	}
}

/*******************************************************************************
 * Routine     : USART3_IRQHandler
 * Function    : USART3 TX and RX IRQ Handle
 ******************************************************************************/
void USART3_IRQHandler(void)
{
/*	if(USART_GetFlagStatus(USART3, USART_IT_TC)!=RESET)
	{
		USART_ClearITPendingBit(USART3,USART_IT_TC);	
		if(tx_len8 == 7)
		{
			tx_len8 = 0;
			GEAR_DIR_RX();
		}
		tx_len8++;
	}*/
		
//USART3 RX Handle-----------------------------------------------------	
	if(USART_GetFlagStatus(USART3, USART_IT_RXNE)!=RESET)								
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);							
		UartGear_RXD_IRQ();
	}
}


