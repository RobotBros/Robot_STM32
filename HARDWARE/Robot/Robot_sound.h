#ifndef  __ROBOT_SOUND_H
#define  __ROBOT_SOUND_H

#include "sys.h"
#include "string.h"

/************************************************************************************
//	nAsrStatus 用来在main主程序中表示程序运行的状态，不是LD3320芯片内部的状态寄存器
//	LD_ASR_NONE:			表示没有在作ASR识别
//	LD_ASR_RUNING：		表示LD3320正在作ASR识别中
//	LD_ASR_FOUNDOK:		表示一次识别流程结束后，有一个识别结果
//	LD_ASR_FOUNDZERO:	表示一次识别流程结束后，没有识别结果
//	LD_ASR_ERROR:			表示一次识别流程中LD3320芯片内部出现不正确的状态
*********************************************************************************/

//LD3220 /RST GPIOC.7 
#define	LD_RST_H()		GPIOC->ODR |= (1<<7)     
#define	LD_RST_L()		GPIOC->ODR &= ~(1<<7) 

//LD3220 /CS GPIOB.12
#define	LD_CS_H()		GPIOB->ODR |= (1<<12)     
#define	LD_CS_L()		GPIOB->ODR &= ~(1<<12) 

typedef struct
{
    u8     	State; 		// ASR state
    u8 	Asr_reg;	// ASR data
    u8 	Asr_busy;
    u8       Asr_acton_cnt;
    u8       Asr_code;
}Sound_type;

///识别码（客户修改处）
#define CODE_NULL		0
#define CODE_LED_OFF	1	 
#define CODE_LED_ON	2	
#define CODE_DUN_XIA	3

///以下三个状态定义用来记录程序是在运行ASR识别还是在运行MP3播放
#define LD_MODE_IDLE			0x00
#define LD_MODE_ASR_RUN		0x08
#define LD_MODE_MP3		 	0x40

///以下五个状态定义用来记录程序是在运行ASR识别过程中的哪个状态
#define LD_ASR_NONE			0x00	//表示没有在作ASR识别
#define LD_ASR_RUNING			0x01	//表示LD3320正在作ASR识别中
#define LD_ASR_FOUNDOK			0x10	//表示一次识别流程结束后，有一个识别结果
#define LD_ASR_FOUNDZERO 		0x11	//表示一次识别流程结束后，没有识别结果
#define LD_ASR_ERROR	 		0x31	//	表示一次识别流程中LD3320芯片内部出现不正确的状态

#define CLK_IN   					25		/* user need modify this value according to clock in */
#define LD_PLL_11				(u8)((CLK_IN/2.0)-1)
#define LD_PLL_MP3_19			0x0f
#define LD_PLL_MP3_1B			0x18
#define LD_PLL_MP3_1D   			(u8)(((90.0*((LD_PLL_11)+1))/(CLK_IN))-1)

#define LD_PLL_ASR_19 			(u8)(CLK_IN*32.0/(LD_PLL_11+1) - 0.51)
#define LD_PLL_ASR_1B 			0x48
#define LD_PLL_ASR_1D 			0x1f

#define MIC_VOL 					0x43

void Robot_Sound_Init(void);
void Robot_Sound_SPI1_Init(void);
void Robot_Sound_Main(void);
u8 Robot_ASR_Handle(u8 code_action);
u8 Robot_ASR_DunXia_Action(void);
void Sound_IRQHandler(void);
void Robot_Sound_TimeT1ms(void);

static u8 LD_AsrAddFixed(void);
static void LD3320_delay(unsigned long uldata);
static u8 RunASR(void);
static void LD_reset(void);
static void LD_AsrStart(void);
static u8 LD_Check_ASRBusyFlag_b2(void);
static u8 spi_send_byte(u8 byte);
static void LD_WriteReg(u8 data1,u8 data2);
static u8 LD_ReadReg(u8 reg_add);
static u8 LD_GetResult(void);
static u8 LD_AsrRun(void);
static void ProcessInt(void);
static void LD_Init_Common(void);
static void LD_Init_ASR(void);

#endif
