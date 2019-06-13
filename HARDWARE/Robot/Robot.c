 /*******************************************************************************
 * FileName£ºRobot.c
 * Function: Robot base function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot.h"

/*******************************************************************************
 * Routine   : Robot_Init
 * Function  : Robot Base Initialization Function
 * Remark   : Base Initialization
 ******************************************************************************/
void Robot_Init(void)
{
	/* Steering Gear Power Initialization ---------------------------------------*/
	Robot_Poweron_Init();
	/* Robot Command Initialization ------------------------------------------*/
	Robot_Command_Init();
	/* Robot Input Current Peak Fault Initialization ------------------------------*/
	Robot_IPK_Fault_Init();
	/* Robot State  Initialization ---------------------------------------------*/
	Robot_State_Init();
	/* Robot System Fault Initialization ---------------------------------------*/
	Robot_Fault_Init();
	/* AD Sample Initialization ----------------------------------------------*/
	Robot_Sample_Init();	
	/* ESP32 Module Initialization --------------------------------------------*/
	Robot_ESP32_Init();
	/* LED I/O Initialization ------------------------------------------------*/
	Robot_LED_Init();			    			 								
	/* Head LED I/O Initialization -------------------------------------------*/
	Robot_Head_LED_PWM_Init();
	/* Steering Gear Action Initialization --------------------------------------*/
	Robot_Steering_Gear_Init();
	/* IMU DMP Software IO Simulation Initialization ----------------------------*/
	//Robot_IMU_SW_Init();
	/* LD3220 Sound Initialization -------------------------------------------*/
	//Robot_Sound_Init();
	/* Motor power on Enable ----------------------------------------------*/
	ROBOT_POWER_ON();
	/* ESP32 Reset Disable ------------------------------------------------*/
	ESP32_ON();

	//Robot_Command_Motor_Enable(ON);	//for ESP32 no motor test
}

/*******************************************************************************
 * Routine   : Robot_Main
 * Function  : Robot base Main loop Function
 * Remark    : 
 ******************************************************************************/
void Robot_Main(void)
{	
	Robot_State_Main();

	Robot_Fault_Main();
	
	Robot_Sample_Main();			
	
	Robot_ESP32_Commu_Main();

	//Robot_IMU_Main();
	
	Robot_LED_Main();
	
	//Robot_Sound_Main();

	Robot_Steering_Gear_Main();
} 

/*******************************************************************************
 * Routine : Robot_Action_TMR_1MS
 * Function: Robot action Timer interrupt Routine
 * Remark: 1ms interrupt
 ******************************************************************************/
void Robot_Action_TMR_1MS(void)
{		
	Robot_State_TimeT1ms();

	Robot_Fault_TimeT1ms();
	
	Robot_Sample_TimeT1ms();
	
	Robot_ESP32Com_TimeT1ms();

	//Robot_IMU_TimeT1ms();
	
	Robot_LED_TimeT1ms();
	
	//Robot_Sound_TimeT1ms();
	
	Robot_Steering_Gear_TimeT1ms();
}


