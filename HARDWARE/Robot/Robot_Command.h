#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include "sys.h"

#define	ROBOT_MOTOR_ENABLE		0
#define	ROBOT_SPEAKER_ENABLE		1
#define	ROBOT_TFCARD_ENABLE		2
#define 	ROBOT_DEBUG_ENABLE		3
#define	ROBOT_IMU_ENABLE			4
#define	ROBOT_VOICE_ENABLE		5

u16 Robot_Command_Out(u8 regist);
void Robot_Command_Init(void);
void Robot_Command_Motor_Enable(u8 enable);
void Robot_Command_Speaker_Enable(u8 enable);
void Robot_Command_Debug_Enable(u8 enable);
void Robot_Command_TFCARD_Enable(u8 enable);
void Robot_Command_IMU_Enable(u8 enable);
void Robot_Voice_Enable(u8 enable);

#endif
