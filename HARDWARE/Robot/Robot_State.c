/*******************************************************************************
 * FileName£ºRobot.c
 * Function: Robot base function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_State.h"

/*******************System State struct********************/
typedef struct
{
	u8 SPEKER_RUNON:1;   		//Bit[00]: Speaker RUN BIT
	u8 MOTOR_RUNON:1;  			//Bit[01]: Motor RUN BIT 
	u8 TFCARD_RUNON:1;  		//Bit[02]: TF Card RUN BIT
}tBits;

typedef union
{
	u8	BYTE;
	tBits		BITS;
}tFunct;
/*****************************************************/

static tFunct FunctStatusBits;				// Robot main function state
static u8 SystemState = 0;					// Robot run state
static u8 *StateReg_Addr[2];				// Robot state register
static u8 T10msFlag_FunctBits = 0;
static u8 Fault_T1s_Counter = 0; 

/*******************************************************************************
 * Routine     : Robot_State_Init
 * Function    : Robot  State  Initialization 
 ******************************************************************************/
void Robot_State_Init(void)
{
	StateReg_Addr[0] = (u8*)&SystemState;
	StateReg_Addr[1] = (u8*)&FunctStatusBits.BYTE;

	SystemState = ROBOT_STATE_IDLE;	
	FunctStatusBits.BYTE = 0;
}

/*******************************************************************************
 * Routine   : Robot_State_Out
 * Function  : Register Read From Robot base
 * Remark   : Return ROBOT_SystemState or ROBOT_FunctStatusBits
 ******************************************************************************/
u8 Robot_State_Out(u8 regist)
{  
	if(regist < 2)	return (*StateReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine     : Robot_State_Main
 * Function    : Robot sysytem state Transition(main loop)
 ******************************************************************************/
void Robot_State_Main(void)
{
	if(T10msFlag_FunctBits)
	{
		T10msFlag_FunctBits = 0;
		Function_Status_Bits();				// Read system function state bit
	}
	
	if(SystemState == ROBOT_STATE_IDLE)	
	{
		Idle_State_Transition();
	}
	else if(SystemState == ROBOT_STATE_RUN)		
	{
		Run_State_Transition();
	}
	else if(SystemState == ROBOT_STATE_FAULT)	
	{
		Fault_State_Transition();
	}
	else if(SystemState == ROBOT_STATE_DEBUG)	
	{
		Debug_State_Transition();
	}
	
}

/*******************************************************************************
 * Routine     : Function_Status_Bits
 ******************************************************************************/
static void Function_Status_Bits(void)
{
	//FunctStatusBits.BYTE = (uint8)Inverter_DoRegRd(Inverter_Status);
}

/*******************************************************************************
 * Routine     : Idle_State_Transition
 ******************************************************************************/
static void Idle_State_Transition(void)
{
	if(Robot_Fault_Out(ROBOT_FAULTBITS))
	{
		SystemState = ROBOT_STATE_FAULT;
	}
	else if(Robot_Command_Out(ROBOT_MOTOR_ENABLE))
	{
		SystemState = ROBOT_STATE_RUN;
	}
	else if(Robot_Command_Out(ROBOT_DEBUG_ENABLE))
	{
		SystemState = ROBOT_STATE_DEBUG;
	}
}

/*******************************************************************************
 * Routine     : Run_State_Transition
 ******************************************************************************/
static void Run_State_Transition(void)
{
	if(Robot_Fault_Out(ROBOT_FAULTBITS))
	{
		SystemState = ROBOT_STATE_FAULT;
	}
	else if(Robot_Command_Out(ROBOT_MOTOR_ENABLE) == OFF)
	{

		SystemState = ROBOT_STATE_IDLE;
	}
}

/*******************************************************************************
 * Routine     : Debug_State_Transition
 ******************************************************************************/
static void Debug_State_Transition(void)
{
	if(Robot_Fault_Out(ROBOT_FAULTBITS))
	{
		SystemState = ROBOT_STATE_FAULT;
	}
	else if(Robot_Command_Out(ROBOT_DEBUG_ENABLE) == OFF)
	{

		SystemState = ROBOT_STATE_IDLE;
	}
}

/*******************************************************************************
 * Routine     : Fault_State_Transition
  * Function   : Clear Fault  
 ******************************************************************************/
static void Fault_State_Transition(void)
{
	static u8 Falutstage = STATE_FALUT_FALSE;

	if(Falutstage == STATE_FALUT_FALSE)
	{
	  	Falutstage = STATE_FALUT_TURE;
		Fault_T1s_Counter = 0;
	}
	else if(Falutstage == STATE_FALUT_TURE)
	{
		if(Fault_T1s_Counter >= FAULT_CLR_TIME)
		{
			 Robot_Fault_Clear();
		}
		
		if(Robot_Fault_Out(ROBOT_FAULTBITS) == 0)
		{
			SystemState = ROBOT_STATE_IDLE;
			Falutstage = STATE_FALUT_FALSE;
		}
	}
}

/*******************************************************************************
 * Routine     : Robot_State_TimeT1ms
 * Function    : 1ms interrupt
 ******************************************************************************/
void Robot_State_TimeT1ms(void)
{
  	static u8 T1msCnt=0;
	static u8 T10msCnt=0;
	static u8 T100msCnt=0;
	
	/*10ms-------------------------------------------------------------------*/
	if(++T1msCnt>=10)
	{
		T1msCnt = 0;
		T10msCnt++;
		
		T10msFlag_FunctBits = 1;
	}
	/*100ms-------------------------------------------------------------------*/
	if(T10msCnt>=10)
	{
		T10msCnt = 0;
		T100msCnt++;  
		
		//T100msFlg_XXX = 1;		// 100ms interupt flag
	}
	/*1000ms------------------------------------------------------------------*/
	if(T100msCnt>=10)
	{
		T100msCnt = 0; 
		
		Fault_T1s_Counter++;
	}   
}


 