#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "Sys.h"

/*Robot state micro define---------------------------------------------------*/
#define	ROBOT_SystemState			0
#define	ROBOT_FunctStatusBits		1

/*Robot system work stage macro define---------------------------------------*/
#define	ROBOT_STATE_IDLE			0
#define	ROBOT_STATE_RUN			1
#define	ROBOT_STATE_FAULT		2
#define	ROBOT_STATE_DEBUG		3

/*macro define------------------------------------------------------------*/
#define	DRIVER_APFC_RUN_BIT			0
#define	DRIVER_MOTOR_RUN_BIT			1
#define	DRIVER_MOTOR_FLXWEAK_BIT	2
#define	DRIVER_MOTOR_TORQUE_BIT		3
#define	DRIVER_DCFAN_RUN_BIT			4
#define	DRIVER_MOTOR_SPDLIM_BIT		5

#define   FAULT_CLR_TIME				60		// 1min
#define   STATE_FALUT_TURE				1
#define   STATE_FALUT_FALSE				0

void Robot_State_Init(void);
u8 Robot_State_Out(u8 regist);
void Robot_State_Main(void);
static void Function_Status_Bits(void);
static void Idle_State_Transition(void);
static void Run_State_Transition(void);
static void Debug_State_Transition(void);
static void Fault_State_Transition(void);
void Robot_State_TimeT1ms(void);

#endif

