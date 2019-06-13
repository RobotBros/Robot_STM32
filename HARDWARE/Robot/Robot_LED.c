/*******************************************************************************
 * FileName£ºRobot_LED.c
 * Function: Robot LED display function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_LED.h"

/*static variable define-----------------------------------------------------------*/ 
static u8 T10msFlag_LED = 0;

/*******************************************************************************
 * Routine: Robot_LED_Init
 * Function: LED Port Initialization
 ******************************************************************************/
void Robot_LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);			// Enable GPIOC clk
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;							// PC13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					// Push pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					//IO speed is 50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);								// Write GPIO config
    	LED_OFF(); 
}

/*******************************************************************************
 * Routine: Robot_LED_Main
 * Function: LEDS Display Function 10ms
 ******************************************************************************/
void Robot_LED_Main(void)
{
	u8 State=0;
	
	if(T10msFlag_LED)
	{
		T10msFlag_LED = 0;
		
		State = Robot_State_Out(ROBOT_SystemState);
		
		switch(State)
		{
			case ROBOT_STATE_IDLE:  LED_IDLE(); break;
			case ROBOT_STATE_RUN:   LED_RUN();  break;
			case ROBOT_STATE_FAULT: LED_FLT();  break;
			case ROBOT_STATE_DEBUG: LED_DEBUG();break;
			default:break;
		}
	}
}

/*==============================================================================
 * Routine: LED_IDLE
 * Function: LED IDLE Display
 *============================================================================*/
static void LED_IDLE(void)
{
	static u16 cnt = 0;
    
	if(++cnt>=100)
	{
		cnt=0;
		LED_INV();
	}
}

/*==============================================================================
 * Routine: LED_RUN
 * Function: LED RUN Display (0.2sON  0.2sOFF)
 *============================================================================*/
static void LED_RUN(void)
{
	LED_ON();
}

/*==============================================================================
 * Routine: LED_FLT
 * Function: LED Fault Display
 *============================================================================*/
static void LED_FLT(void)
{
	u8 FLT_Code;
	
	FLT_Code = (u8)Robot_Fault_Out(ROBOT_FAULTCODE);
	//T_Code =  (uint8)Get_FltCode_Custm();					//Custemer Fault code
	LED_FLT_Code(FLT_Code);

}

/*==============================================================================
 * Routine: LED_DEBUG
 * Function: LED Debug Display
 *============================================================================*/
static void LED_DEBUG(void)
{
	static u16 cnt = 0;
    
	if(++cnt>=50)
	{
		cnt=0;
		LED_INV();
	}
}

/*------------------------------------------------------------------------------
 * Routine: LED_FLT_Code
 * Function: Fault Code LED Display
 *----------------------------------------------------------------------------*/
static void LED_FLT_Code(u8 FltCode)
{
	u16 tmp16;		
	static u16 i=0;
	static u8 LEDStep=0;
	static u8 LongFlashTimes = 0;
	static u8 ShortFlashTimes = 0;

	switch(LEDStep)
	{
		case 0: 
		{
			LED_OFF();
			LEDStep = 1;
			i = 0;
			tmp16 = (u16)FltCode;
			LongFlashTimes = tmp16/5;
			ShortFlashTimes = tmp16%5;
			break;
		}
		case 1: 
		{
			i++;
			if(i>=300)
			{
				i = 0;
				LEDStep = 2;
				LED_ON();
			}
			break;
		}
		case 2: 
		{
			if(LongFlashTimes == 0)
			{
				LEDStep = 3;
				break;
			}
			i++;
			if(i<=100)		
			{
				LED_ON();
			}
			else if(i<=200)	
			{
				LED_OFF();
			}
			else	
			{
				i = 0;        					
				LongFlashTimes--;
			}
			break;
		}
		case 3: 
		{
			if(ShortFlashTimes == 0)
			{
				LEDStep = 0;
				break;
			}
	
			i++;
			if(i<=50)			
			{
				LED_ON();
			}
			else if(i<=100)	
			{
				LED_OFF();
			}
			else	
			{				
				i = 0;
				ShortFlashTimes--;
			}
			break;
		}
		default:break;
	}
}
	
/*******************************************************************************
 * Routine: Robot_LED_TimeT1ms
 * Function: LEDS Function Timebase
 ******************************************************************************/
void Robot_LED_TimeT1ms(void)
{
    static u8 T1msCnt=0;

	/*10ms-------------------------------------------------------------------*/
	if(++T1msCnt>=10)
	{
		T1msCnt = 0;
    	T10msFlag_LED = 1;
	}
}
