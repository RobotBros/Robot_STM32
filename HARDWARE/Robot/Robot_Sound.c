/*******************************************************************************
 * FileName：Robot_Sound.c
 * Function:  Robot Sound function - SPI2
 * Designer: Fishcan/2018.11.14
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_Sound.h"

/*static variable define----------------------------------------------------------*/ 
static Sound_type Sound_Asr;
static u8  TmsFlag_ASR_handle = 0;		// ASR timer
static u8  TmsFlag_sound = 0;
static u8  TsFlag_ASR_ADJ = 0;				// ASR adjustment
u8 nLD_Mode = LD_MODE_IDLE;			//用来记录当前是在进行ASR识别还是在播放MP3
u8 ucRegVal;

tGear_TX_Frame Gear_tx_frame;

/*******************************************************************************
 * Routine     : Robot_Sound_Init
 * Function    : Robot Sound Interface Initialization
 ******************************************************************************/
void Robot_Sound_Init(void)
{	
/*variable initial----------------------------------------------------------------*/ 	
	Sound_Asr.State = LD_ASR_NONE;
	Sound_Asr.Asr_reg = 0;
	Sound_Asr.Asr_acton_cnt = 0;
	Sound_Asr.Asr_busy = 0;
	Sound_Asr.Asr_code = 0;
/*SPI2 initial-------------------------------------------------------------------*/ 	
	Robot_Sound_SPI1_Init();
/*LD3320 reset-----------------------------------------------------------------*/ 
	LD_reset();
}

/*******************************************************************************
 * Routine     : Robot_Sound_SPI1_Init
 * Function    : Robot Sound SPI1 Initialization
 ******************************************************************************/
void Robot_Sound_SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	

	SPI_I2S_DeInit(SPI1); 	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);
/*LD3220 RST GPIOC.7 Output--------------------------------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	LD_RST_H();
/*LD3220 CS GPIOB.12 Output---------------------------------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	LD_CS_H();
	
/*LD3220 INT GPIOC.6 Input-----------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource6);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line6;																		
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	
	//NVIC_SetPriority(EXTI9_5_IRQn,6);							//IRQ6									
	//NVIC_EnableIRQ(EXTI9_5_IRQn);
	
/*LD3220 , GPIOA.5 SCK GPIOA.6 MISO GPIOA.7 MOSI, ---------------------------------*/ 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	SPI_Cmd(SPI1, DISABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	//设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;					//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	//定义波特率预分频的值:波特率预分频值为2  36/32=2.25MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

/*SPI1 Interrupt------------------------------------------------------------*/													
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);				 
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE,   ENABLE);	
	//SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
	//NVIC_SetPriority(SPI1_IRQn,2);									
	//NVIC_EnableIRQ(SPI1_IRQn);									
  	               
    	SPI_Cmd(SPI1, ENABLE); 		
}

/******************************************************************************
 * Routine     : Robot_Sound_Main
 * Function    : Robot Sound ASR main Handle
 ******************************************************************************/
void Robot_Sound_Main(void)
{
	u8 state,s_enable;

	state = Robot_State_Out(ROBOT_SystemState);
	s_enable = Robot_Command_Out(ROBOT_VOICE_ENABLE);

	
	if(s_enable) {								// check Voice module is it here
	if(TsFlag_ASR_ADJ)
	{
		TsFlag_ASR_ADJ = 0;
		if(state == ROBOT_STATE_RUN) 
		{
			LD_reset();
			Sound_Asr.State = LD_ASR_NONE;
		}
	}
	
	if(TmsFlag_sound)
	{
		TmsFlag_sound = 0;									// 1ms action
		
		if(state == ROBOT_STATE_RUN)
		{
			switch(Sound_Asr.State)
			{
				case LD_ASR_RUNING:  break;					// Busy
				case LD_ASR_ERROR:	 LD_reset(); Sound_Asr.State = LD_ASR_NONE; break;			
				case LD_ASR_NONE:
						if(TmsFlag_ASR_handle)
						{
							TmsFlag_ASR_handle = 0;		// 500ms ASR
							
							if(!Sound_Asr.Asr_busy)
							{
								Sound_Asr.State = LD_ASR_RUNING;
								if (RunASR()==0)						//启动一次ASR识别流程：ASR初始化，ASR添加关键词语，启动ASR运算
								{		
									Sound_Asr.State = LD_ASR_ERROR;
								}
							}
						}
						break;
				
				case LD_ASR_FOUNDOK:
						if(!Sound_Asr.Asr_busy)
						{
							Sound_Asr.Asr_reg = LD_GetResult();	//一次ASR识别流程结束，去取ASR识别结果
						}
						
						if(Robot_ASR_Handle(Sound_Asr.Asr_reg))
						{
							Sound_Asr.State = LD_ASR_NONE;
						}
					
						break;
				
				case LD_ASR_FOUNDZERO: Sound_Asr.State = LD_ASR_NONE; break;
				default: Sound_Asr.State = LD_ASR_NONE; break;
			}
		}
	}
	}
}

/******************************************************************************
 * Routine     : Robot_ASR_Handle
 * Function    : Robot Sound ASR Handle
 ******************************************************************************/
u8 Robot_ASR_Handle(u8 code_action)
{	
	u8 result = 0;
	
	switch(code_action)		   			
	{
		case CODE_LED_OFF:	Robot_Head_LED_PWM_Percentage(0); Sound_Asr.Asr_busy = 0; return(1); break;			
		case CODE_LED_ON:	Robot_Head_LED_PWM_Percentage(50); Sound_Asr.Asr_busy = 0; return(1); break;			
		case CODE_DUN_XIA: 
			Sound_Asr.Asr_busy = 1;
			
			result = Robot_ASR_DunXia_Action();
			if(result) {Sound_Asr.Asr_busy = 0; return(1);}
			else return(0);
			break;		
		default:break;
	}	
}

/*******************************************************************************
 * Routine     : Robot_ASR_DunXia_Action
 * Function    : ASR Dun xia action handle
 ******************************************************************************/
u8 Robot_ASR_DunXia_Action(void)
{	
	static u8 m1ang=0,m2ang=0,m3ang=0,m4ang=0,m5ang=0,m6ang=0,m7ang=0,m8ang=0,m9ang=0,m10ang=0,m11ang=0,m12ang=0,m13ang=0,m14ang=0,m15ang=0,m16ang=0;

	if(Gear_tx_frame.TX_done == 1)
	{
		Gear_tx_frame.TX_done = 0;
		
		Sound_Asr.Asr_acton_cnt++;
		
		switch(Sound_Asr.Asr_acton_cnt)				// action step
		{
//STEP 1		
			case 1: 
				Gear_tx_frame.Sel_uni_comb = TX_UNI;
				Gear_tx_frame.Command = ACTION_TIME_COMMAND;
				Gear_tx_frame.Id = 10;				//motor ID
				m10ang = 10;
				Gear_tx_frame.Action_times= 30;		//ms
				Gear_tx_frame.Angle = m10ang;		//angle
				Robot_Gear_Angle_Cla(MINUS);		// θ-  angle
				break;
			case 2:
				Gear_tx_frame.Id = 15;
				m15ang = 10;
				Gear_tx_frame.Action_times= 30;
				Gear_tx_frame.Angle = m15ang;	
				Robot_Gear_Angle_Cla(ADD);		// θ+  angle
				break;
			case 3:
				Gear_tx_frame.Id = 9;				//motor ID
				m9ang =  10;
				Gear_tx_frame.Action_times= 30;		//ms
				Gear_tx_frame.Angle = m9ang;		//angle
				Robot_Gear_Angle_Cla(ADD);			// θ+ angle
				break;
			case 4:
				Gear_tx_frame.Id = 14;
				m14ang = 10;
				Gear_tx_frame.Action_times= 30;
				Gear_tx_frame.Angle = m14ang;	
				Robot_Gear_Angle_Cla(MINUS);		// θ-  angle
				break;
//STEP 2
/*			case 5: 
				Gear_tx_frame.Id = 10;				//motor ID
				m10ang = m10ang + 10;
				Gear_tx_frame.Action_times= 30;		//ms
				Gear_tx_frame.Angle = m10ang;		//angle
				Robot_Gear_Angle_Cla(MINUS);		// θ-  angle
				break;
			case 6:
				Gear_tx_frame.Id = 15;
				m15ang = m15ang + 10;
				Gear_tx_frame.Action_times= 30;
				Gear_tx_frame.Angle = m15ang;	
				Robot_Gear_Angle_Cla(ADD);		// θ+  angle
				break;
			case 7:
				Gear_tx_frame.Id = 9;				//motor ID
				m9ang =  m9ang + 10;
				Gear_tx_frame.Action_times= 30;		//ms
				Gear_tx_frame.Angle = m9ang;		//angle
				Robot_Gear_Angle_Cla(ADD);			// θ+ angle
				break;
			case 8:
				Gear_tx_frame.Id = 14;
				m14ang = m14ang + 10;
				Gear_tx_frame.Action_times= 30;
				Gear_tx_frame.Angle = m14ang;	
				Robot_Gear_Angle_Cla(MINUS);		// θ-  angle
				break;
*/
			default:break;
		}

		
		if(Sound_Asr.Asr_acton_cnt >= 4)  {Sound_Asr.Asr_acton_cnt = 0; return(1);}
		else return(0);
	}
		
	//memset(Esp32_rx_buf,0,sizeof(Esp32_rx_buf));
}

/******************************************************************************
 * Routine     : Sound_IRQHandler
 * Function    : SPI2 IRQ ASR finish Interrupt Handle
 ******************************************************************************/
void Sound_IRQHandler(void)
{	
	ProcessInt(); 
}

/*******************************************************************************
 * Routine     : Robot_Sound_TimeT1ms
 * Function    : ESP32 Sound ASR 1ms interrupt
 ******************************************************************************/
void Robot_Sound_TimeT1ms(void)
{
    static u16 TCnt=0;	
    static u16 T1msCnt=0;
    static u16 Thalf1sCnt=0;	
    static u16 T1sCnt = 0;

/*4ms---------------------------------------------------------------------*/
	if(++TCnt >=4)
	{
		TCnt = 0;
		TmsFlag_sound = 1;		// 4ms
	}
/*500ms-------------------------------------------------------------------*/
	if(++T1msCnt >= 500)
	{
		T1msCnt = 0;
		Thalf1sCnt++;
		
		TmsFlag_ASR_handle = 1;
	}
/*1s-----------------------------------------------------------------------*/
	if(Thalf1sCnt >= 2)
	{
		Thalf1sCnt = 0;
		T1sCnt++;
	}
/*30s-----------------------------------------------------------------------*/
	if(T1sCnt >= 30)
	{
		T1sCnt = 0;

		TsFlag_ASR_ADJ = 1;
	}	
}

//--------------------------------------Base funtion---------------------------------------//
static u8 LD_AsrAddFixed(void)
{
	u8 k, flag;
	u8 nAsrAddLength;
	#define DATE_A 3    					//数组二维数值
	#define DATE_B 10					//数组一维数值
	//添加关键词，用户修改
	u8  sRecog[DATE_A][DATE_B] = {
	 								"guan deng",\
									"kai deng",\
									"dun xia",\
								    };	
	
	u8  pCode[DATE_A] = {
	 						CODE_LED_OFF,	\
							CODE_LED_ON,	\
							CODE_DUN_XIA,	\
						  };	//添加识别码，用户修改
	flag = 1;
	for (k=0; k<DATE_A; k++)
	{			
		if(LD_Check_ASRBusyFlag_b2() == 0)
		{
			flag = 0;
			break;
		}

		LD_WriteReg(0xc1, pCode[k] );
		LD_WriteReg(0xc3, 0);
		LD_WriteReg(0x08, 0x04);
		LD3320_delay(1);
		LD_WriteReg(0x08, 0x00);
		LD3320_delay(1);

		for (nAsrAddLength=0; nAsrAddLength<DATE_B; nAsrAddLength++)
		{
			if (sRecog[k][nAsrAddLength] == 0)
				break;
			LD_WriteReg(0x5, sRecog[k][nAsrAddLength]);
		}
		LD_WriteReg(0xb9, nAsrAddLength);
		LD_WriteReg(0xb2, 0xff);
		LD_WriteReg(0x37, 0x04);
	}	 
	return flag;
}

static void LD3320_delay(unsigned long uldata)
{
	unsigned int i  =  0;
	unsigned int j  =  0;
	unsigned int k  =  0;
	for (i=0;i<5;i++)
	{
		for (j=0;j<uldata;j++)
		{
			k = 200;
			while(k--);
		}
	}
}

static u8 RunASR(void)
{
	u8 i=0;
	u8 asrflag=0;
	
	for (i=0; i<5; i++)		//防止由于硬件原因导致LD3320芯片工作不正常，所以一共尝试5次启动ASR识别流程
	{
		//LD_AsrStart();			//初始化ASR
		//LD3320_delay(100);
		//if (LD_AsrAddFixed()==0)	//添加关键词语到LD3320芯片中
		//{
			//LD_reset();				//LD3320芯片内部出现不正常，立即重启LD3320芯片
			//LD3320_delay(50);	//并从初始化开始重新ASR识别流程
			//continue;
		//	return asrflag;
		//}
		//LD3320_delay(10);
		if (LD_AsrRun() == 0)
		{
			//LD_reset();			 //LD3320芯片内部出现不正常，立即重启LD3320芯片
			//LD3320_delay(50);//并从初始化开始重新ASR识别流程
			continue;
			//return asrflag;
		}
		asrflag=1;
		break;						//ASR流程启动成功，退出当前for循环。开始等待LD3320送出的中断信号
	}	
	return asrflag;
}

static void LD_reset(void)
{
	u8 reg = 0;

	LD_RST_H();
	LD3320_delay(100);
	LD_RST_L();
	LD3320_delay(100);
	LD_RST_H();
	LD3320_delay(100);
	LD_CS_L();
	LD3320_delay(100);
	LD_CS_H();		
	LD3320_delay(100);

/*LD3320 module check----------------------------------------------------------*/
	reg = 0;
	//reg = LD_ReadReg(0x06);
	//reg = LD_ReadReg(0x06);
	//reg = LD_ReadReg(0x35);
	reg = LD_ReadReg(0xB3);	
	if(reg == 0xff) 
	{
		Robot_Voice_Enable(ON);
/*LD3320 REG Initial-------------------------------------------------------------*/ 
		LD_Init_ASR();
		LD3320_delay(100);
/*LD3320 ADD Word-------------------------------------------------------------*/ 
		if (LD_AsrAddFixed()==0)		Sound_Asr.State = LD_ASR_ERROR;
	}
	
	else	 {Robot_Voice_Enable(OFF);}
}

static void LD_AsrStart(void)
{
	LD_Init_ASR();
} 


u8 LD_Check_ASRBusyFlag_b2(void)
{
	u8 j;
	u8 flag = 0;
	
	for (j=0; j<10; j++)
	{
		if (LD_ReadReg(0xb2) == 0x21)
		{
			flag = 1;
			break;
		}
		LD3320_delay(10);		
	}
	return flag;
}
///中间层end


///寄存器操作
static u8 spi_send_byte(u8 byte)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2,byte);
	while (SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI2);
}

static void LD_WriteReg(u8 data1,u8 data2)
{
	LD_CS_L();
	spi_send_byte(0x04);
	spi_send_byte(data1);
	spi_send_byte(data2);
	LD_CS_H();
}

static u8 LD_ReadReg(u8 reg_add)
{
	u8 i;
	
	LD_CS_L();
	spi_send_byte(0x05);
	spi_send_byte(reg_add);
	i=spi_send_byte(0x00);
	LD_CS_H();
	return(i);
}

static u8 LD_GetResult(void)
{
	return LD_ReadReg(0xc5);
}

static u8 LD_AsrRun(void)
{
	LD_WriteReg(0x35, MIC_VOL);
	LD_WriteReg(0x1C, 0x09);
	LD_WriteReg(0xBD, 0x20);
	LD_WriteReg(0x08, 0x01);
	LD3320_delay( 5 );
	LD_WriteReg(0x08, 0x00);
	LD3320_delay( 5);

	if(LD_Check_ASRBusyFlag_b2() == 0)
	{
		return 0;
	}

	LD_WriteReg(0xB2, 0xff);	
	LD_WriteReg(0x37, 0x06);
	LD_WriteReg(0x37, 0x06);
	LD3320_delay(5);
	LD_WriteReg(0x1C, 0x0b);
	LD_WriteReg(0x29, 0x10);
	LD_WriteReg(0xBD, 0x00);   
	return 1;
}

static void ProcessInt(void)
{
	u8 nAsrResCount=0;

	ucRegVal = LD_ReadReg(0x2B);

// 语音识别产生的中断
//（有声音输入，不论识别成功或失败都有中断）
	LD_WriteReg(0x29,0) ;
	LD_WriteReg(0x02,0) ;

	if((ucRegVal & 0x10) && LD_ReadReg(0xb2)==0x21 && LD_ReadReg(0xbf)==0x35)		
	{	 
			nAsrResCount = LD_ReadReg(0xba);

			if(nAsrResCount>0 && nAsrResCount<=4) 
			{
				Sound_Asr.State = LD_ASR_FOUNDOK; 				
			}
			else
			{
				Sound_Asr.State = LD_ASR_FOUNDZERO;
			}	
	}
	else
	{
		Sound_Asr.State = LD_ASR_FOUNDZERO;			//执行没有识别
	}

	LD_WriteReg(0x2b,0);
	LD_WriteReg(0x1C,0);//写0:ADC不可用
	LD_WriteReg(0x29,0);
	LD_WriteReg(0x02,0);
	LD_WriteReg(0x2B,0);
	LD_WriteReg(0xBA,0);	
	LD_WriteReg(0xBC,0);	
	LD_WriteReg(0x08,1);//清除FIFO_DATA
	LD_WriteReg(0x08,0);//清除FIFO_DATA后 再次写0
}

static void LD_Init_Common(void)
{
	LD_ReadReg(0x06);  
	LD_WriteReg(0x17, 0x35); 
	LD3320_delay(5);
	LD_ReadReg(0x06);  

	LD_WriteReg(0x89, 0x03);  
	LD3320_delay(5);
	LD_WriteReg(0xCF, 0x43);   
	LD3320_delay(5);
	LD_WriteReg(0xCB, 0x02);
	
	/*PLL setting*/
	LD_WriteReg(0x11, LD_PLL_11);       
	if (nLD_Mode == LD_MODE_MP3)
	{
		LD_WriteReg(0x1E, 0x00); 
		LD_WriteReg(0x19, LD_PLL_MP3_19);   
		LD_WriteReg(0x1B, LD_PLL_MP3_1B);   
		LD_WriteReg(0x1D, LD_PLL_MP3_1D);
	}
	else
	{
		LD_WriteReg(0x1E,0x00);
		LD_WriteReg(0x19, LD_PLL_ASR_19); 
		LD_WriteReg(0x1B, LD_PLL_ASR_1B);		
	  LD_WriteReg(0x1D, LD_PLL_ASR_1D);
	}
	LD3320_delay(5);
	
	LD_WriteReg(0xCD, 0x04);
	LD_WriteReg(0x17, 0x4c); 
	LD3320_delay(1);
	LD_WriteReg(0xB9, 0x00);
	LD_WriteReg(0xCF, 0x4F); 
	LD_WriteReg(0x6F, 0xFF); 
}

static void LD_Init_ASR(void)
{
	nLD_Mode=LD_MODE_ASR_RUN;
	LD_Init_Common();

	LD_WriteReg(0xBD, 0x00);
	LD_WriteReg(0x17, 0x48);	
	LD3320_delay(5);
	LD_WriteReg(0x3C, 0x80);    
	LD_WriteReg(0x3E, 0x07);
	LD_WriteReg(0x38, 0xff);    
	LD_WriteReg(0x3A, 0x07);
	LD_WriteReg(0x40, 0);          
	LD_WriteReg(0x42, 8);
	LD_WriteReg(0x44, 0);    
	LD_WriteReg(0x46, 8); 
	LD3320_delay( 1 );
}

