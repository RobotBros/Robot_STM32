 /*******************************************************************************
 * FileName£ºRobot_Command.c
 * Function:  Robot function ctrl enable
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_Command.h"

/*static variable define----------------------------------------------------------*/ 
static u16 Motor_Command_Enable = 0;
static u16 Speaker_Command_Enable = 0;
static u16 Tfcard_Command_Enable = 0;
static u16 Debug_Command_Enable = 0;			// Debug mode Robot Action ctrl by ESP32
static u16 Imu_Command_Enable = 0;	
static u16 Voice_Command_Enable = 0;
static u16 *CommandReg_Addr[6];


/*******************************************************************************
 * Routine   : Robot_Command_Out
 * Function  : Register Read From Robot Base Driver - STM32
 ******************************************************************************/
u16 Robot_Command_Out(u8 regist)
{
	if(regist < 7)	return (*CommandReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine     : Robot_Command_Init
 * Function    : Robot Command Control Initialization
 ******************************************************************************/
void Robot_Command_Init(void)
{
	CommandReg_Addr[0] = &Motor_Command_Enable;
	CommandReg_Addr[1] = &Speaker_Command_Enable;
	CommandReg_Addr[2] = &Tfcard_Command_Enable;
	CommandReg_Addr[3] = &Debug_Command_Enable;
	CommandReg_Addr[4] = &Imu_Command_Enable;
	CommandReg_Addr[5] = &Voice_Command_Enable;
	
	Motor_Command_Enable = 0;	
	Speaker_Command_Enable = 0;
	Tfcard_Command_Enable = 0;
	Debug_Command_Enable = 0;
	Imu_Command_Enable = 0;
	Voice_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_Motor_Enable
 * Function    : Robot Motor Enable
 ******************************************************************************/
void Robot_Command_Motor_Enable(u8 enable)
{
	if(enable)	{Motor_Command_Enable = 1;}
	else		Motor_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_Speaker_Enable
 * Function    : Robot Speaker Enable
 ******************************************************************************/
void Robot_Command_Speaker_Enable(u8 enable)
{
	if(enable)	Speaker_Command_Enable = 1;
	else		Speaker_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_Debug_Enable
 * Function    : Robot Debug mode Enable
 ******************************************************************************/
void Robot_Command_Debug_Enable(u8 enable)
{
	if(enable)	Tfcard_Command_Enable = 1;
	else		Tfcard_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_TFCARD_Enable
 * Function    : Robot TF Card Enable
 ******************************************************************************/
void Robot_Command_TFCARD_Enable(u8 enable)
{
	if(enable)	Debug_Command_Enable = 1;
	else		Debug_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_TFCARD_Enable
 * Function    : Robot TF Card Enable
 ******************************************************************************/
void Robot_Command_IMU_Enable(u8 enable)
{
	if(enable)	Imu_Command_Enable = 1;
	else		Imu_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Voice_Enable
 * Function    : Robot Voice Module Enable
 ******************************************************************************/
void Robot_Voice_Enable(u8 enable)
{
	if(enable)	Voice_Command_Enable = 1;
	else		Voice_Command_Enable = 0;
}

 //END*************************************************************************/
 
