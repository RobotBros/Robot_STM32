/*******************************************************************************
 * FileName£ºRobot_Fault.c
 * Function: Protect Control and fault function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_Fault.h"

typedef struct
{
	u32 FLASH_DRV_FLT:1;		//Bit[00]: Flash of Driver fault
	u32 NA1:1;					//Bit[01]: NA1
	u32 NA2:1;					//Bit[02]: NA2
	u32 COMMUNICATEFLT:1;		//Bit[03]: Communicaton to upper Control fault
	u32 NA4:1;					//Bit[04]: NA4
	u32 NA5:1;					//Bit[05]: NA5
	u32 VDC_BAT_OVER:1;		//Bit[06]: Vbat Over Voltage fault
	u32 VDC_BAT_UNDR:1;		//Bit[07]: Vbat Under Voltage fault
	u32 PWR_INPUT_OVER:1;		//Bit[08]: Power Input Over Load fault
	u32 IDC_INPUT_OVER:1;		//Bit[09]: Battery Input Over Current fault -software
	u32 IPK_INPUT_OVER:1;		//Bit[10]: Battery Input Over Current fault -hardware
	u32 NA11:1;					//Bit[11]: NA11
	u32 NA12:1;					//Bit[12]: NA12
	u32 NA13:1;					//Bit[13]: NA13
	u32 NA14:1;					//Bit[14]: NA14
	u32 NA15:1;					//Bit[15]: NA15
	u32 NA16:1;					//Bit[16]: NA16
	u32 NA17:1;					//Bit[17]: NA17
	u32 NA18:1;					//Bit[18]: NA18
	u32 NA19:1;					//Bit[19]: NA19
}tBits;

typedef union
{
	u32	WORD;
	tBits		BITS;
}tFault;

/*Static Variable define------------------------------------------------------*/
static tFault Robot_FaultBits;
static u8 T10msFlag_Protect = 0;
static u32 FaultCode = 0;
static u32 *FaultReg_Addr[2];

/*******************************************************************************
 * Routine   : Robot_Fault_Out
 * Function  : Read Robot Fault flag
 ******************************************************************************/
u32 Robot_Fault_Out(u8 regist)
{  
	if(regist < 6)	return (*FaultReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine     : Robot_Fault_Init
 * Function    : Robot Fault Initialization
 ******************************************************************************/
void Robot_Fault_Init(void)
{
	FaultReg_Addr[0] = (u32*)&Robot_FaultBits.WORD;
	FaultReg_Addr[1] = (u32*)&FaultCode;

  	FaultCode = 0;
	Robot_FaultBits.WORD = 0;
}

/*******************************************************************************
 * Routine     : Robot_Fault_Clear
 * Function    : Robot Fault Clear
 ******************************************************************************/
void Robot_Fault_Clear(void)
{
	Robot_FaultBits.WORD = 0;
	FaultCode = 0;
}

/*******************************************************************************
 * Routine     : Robot_Protect
 * Function    : Robot_Protection(main loop)
 ******************************************************************************/
void Robot_Fault_Main(void)
{
	Robot_Fault_Detect();
  	Robot_Fault_Process();
}

/*******************************************************************************
 * Routine     : Robot_Fault_Process
 * Function    : Robot Fault state to process the power mosfet
*******************************************************************************/
void Robot_Fault_Process(void)
{
	if(Robot_FaultBits.WORD)
	{
		ROBOT_POWER_OFF();
		Robot_Head_LED_PWM_Percentage(0);
	}
}

/*******************************************************************************
 * Routine     : Robot_Fault_Detect
 * Function    : Robot  Fault detect 
 ******************************************************************************/
static void Robot_Fault_Detect(void)
{
	if(T10msFlag_Protect)
	{
		T10msFlag_Protect = 0;
		
		Flash_Fault_Detect();
		
		VBAT_Fault_Detect();
		
		IDC_Fault_Detect();

		IPK_Fault_Detect();
		
		//PWR_Fault_Detect();
		
		Robot_Fault_Code_Detect();
	}
}

/*******************************************************************************
 * Routine     : Flash_Fault_Detect
 * Function    : Flash parameters Fault Detection
*******************************************************************************/
static void Flash_Fault_Detect(void)
{
	//if(Robot_Flash_Out(FLASH_CheckSumFault))			// Read data checksum detect
	//{
	//	Robot_FaultBits.BITS.FLASH_DRV_FLT = 1;
	//}
}

/*------------------------------------------------------------------------------
 * Routine     : VBAT_Fault_Detect
 * Function    : Battery voltage over and under voltage Fault Detection
 *----------------------------------------------------------------------------*/
static void VBAT_Fault_Detect(void)
{
	static u8 cnt1_times = 0;
	static u8 cnt2_times = 0;
	static u16 Vbat_RMS;


	Vbat_RMS = Robot_Sample_Out(ADC1_VBAT_BYTE, FILITER_DISABLE);		// NO LPF fliter
		
	if(Vbat_RMS >= VBAT_OVER_VOLTAGE)
	{
		if(++cnt1_times>=5)												// 5ms 
		{
			cnt1_times = 0;
			Robot_FaultBits.BITS.VDC_BAT_OVER = 1;
		}
	}		
	else cnt1_times = 0;

	if(Vbat_RMS <= VBAT_UNDER_VOLTAGE)
	{
		if(++cnt2_times>=5)
		{
			cnt2_times = 0;
			Robot_FaultBits.BITS.VDC_BAT_UNDR = 1;	
		}
	}		
	else cnt2_times= 0;
}

/*******************************************************************************
 * Routine     : IDC_Fault_Detect
 * Function    : IDC over Current Fault Detection-softwear
******************************************************************************/
static void IDC_Fault_Detect(void)
{
	static u8 cnt_times = 0;	
	static u16 Idc_RMS;
	
	Idc_RMS = Robot_Sample_Out(ADC1_IBUS_BYTE, FILITER_ENABLE);
	
	if(Idc_RMS >= VBAT_OVER_CURRENT)
	{
		if(++cnt_times>=20)
		{
			cnt_times = 0;
			Robot_FaultBits.BITS.IDC_INPUT_OVER = 1;
		}
	}
	else cnt_times = 0;
}

/*******************************************************************************
 * Routine     : IPK_Fault_Detect
 * Function    : IDC Peak over Current Fault Detection=heardwear
******************************************************************************/
static void IPK_Fault_Detect(void)
{
	if(Robot_Hardware_Fault_Out())
	{
		Robot_FaultBits.BITS.IPK_INPUT_OVER = 1;
	}
}

/*******************************************************************************
 * Routine     : PWR_Fault_Detect
 * Function    : Power over Load Fault Detection
 ******************************************************************************/
/*static void PWR_Fault_Detect(void)
{
	static uint8 cnt_times = 0;	
	uint16 Power_AVG;
	
	Power_AVG = Robot_Sample_Out(ROBOT_POWERAVG);
	
	if(Power_AVG >= VBAT_OVER_POWER)
	{
		if(++cnt_times>=100)
		{
			cnt_times = 0;
			Robot_FaultBits.BITS.PWR_INPUT_OVER = 1;
			}
		}
		else cnt_times = 0;
	}
}*/

/*******************************************************************************
 * Routine     : Robot_Fault_Code_Detect
 * Function    : Robot FaultCode Detection
 ******************************************************************************/
static void Robot_Fault_Code_Detect(void)
{
	if(Robot_FaultBits.WORD == 0)					FaultCode = 0;
	else if(Robot_FaultBits.BITS.FLASH_DRV_FLT)	FaultCode = 1;
	else if(Robot_FaultBits.BITS.NA1)				FaultCode = 2;
	else if(Robot_FaultBits.BITS.NA2)				FaultCode = 3;
	else if(Robot_FaultBits.BITS.COMMUNICATEFLT)	FaultCode = 4;
	else if(Robot_FaultBits.BITS.NA4)				FaultCode = 5;
	else if(Robot_FaultBits.BITS.NA5)				FaultCode = 6;
	else if(Robot_FaultBits.BITS.VDC_BAT_OVER)	FaultCode = 7;
	else if(Robot_FaultBits.BITS.VDC_BAT_UNDR)	FaultCode = 8;
	else if(Robot_FaultBits.BITS.PWR_INPUT_OVER)	FaultCode = 9;
	else if(Robot_FaultBits.BITS.IDC_INPUT_OVER)	FaultCode = 10;
	else if(Robot_FaultBits.BITS.IPK_INPUT_OVER)	FaultCode = 11;
	else if(Robot_FaultBits.BITS.NA11)				FaultCode = 12;
	else if(Robot_FaultBits.BITS.NA12)				FaultCode = 13;
	else if(Robot_FaultBits.BITS.NA13)				FaultCode = 14;
	else if(Robot_FaultBits.BITS.NA14)				FaultCode = 15;
	else if(Robot_FaultBits.BITS.NA15)				FaultCode = 16;
	else if(Robot_FaultBits.BITS.NA16)				FaultCode = 17;
	else if(Robot_FaultBits.BITS.NA17)				FaultCode = 18;
	else if(Robot_FaultBits.BITS.NA18)				FaultCode = 19;
	else if(Robot_FaultBits.BITS.NA19)				FaultCode = 20;
}

/*******************************************************************************
 * Routine     : Robot_Fault_TimeT1ms
 * Function    : Robot Protection timebase(1ms)
 ******************************************************************************/
void Robot_Fault_TimeT1ms(void)
{
    static u8 T1msCnt=0;

	/*10ms-------------------------------------------------------------------*/
	if(++T1msCnt>=10)
	{
		T1msCnt = 0;

		T10msFlag_Protect = 1;
	}
}

/********************************************************************************
 * Routine     : Get_FltCode_Custm
 * Function    : Get Robot Fault Code for Custumer Inetrface 
 ********************************************************************************/
u16 Get_FltCode_Custm(void)
{
 	u16 tmp16;

	if(FaultCode == 0)	tmp16 = 0;
	if(FaultCode == 1)	tmp16 = 25;
	if(FaultCode == 2)	tmp16 = 7;
	if(FaultCode == 3)	tmp16 = 9;
	if(FaultCode == 4)	tmp16 = 22;
	if(FaultCode == 5)	tmp16 = 10;
	if(FaultCode == 6)	tmp16 = 11;
	if(FaultCode == 7)	tmp16 = 1;
	if(FaultCode == 8)	tmp16 = 2;
	if(FaultCode == 9)	tmp16 = 17;
	if(FaultCode == 10)	tmp16 = 12;
	if(FaultCode == 11)	tmp16 = 3;
	if(FaultCode == 12)	tmp16 = 6;
	if(FaultCode == 13)	tmp16 = 4;
	if(FaultCode == 14)	tmp16 = 14;
	if(FaultCode == 15)	tmp16 = 15;
	if(FaultCode == 16)	tmp16 = 16;
	if(FaultCode == 17)	tmp16 = 21;
	if(FaultCode == 18)	tmp16 = 18;
	if(FaultCode == 19)	tmp16 = 19;
	if(FaultCode == 20)	tmp16 = 20; 
	
	return(tmp16);
}

