#ifndef ROBOT_FAULT_H
#define	ROBOT_FAULT_H

#include "sys.h"

/*Driver_Fault_Out(regist) regist addr define---------------------------------*/
#define	ROBOT_FAULTBITS	0
#define	ROBOT_FAULTCODE	1

void Robot_Fault_Init(void);
u32 Robot_Fault_Out(u8 regist);
void Robot_Fault_Clear(void);
void Robot_Fault_Main(void);
void Robot_Fault_Process(void);
static void Robot_Fault_Detect(void);
static void Flash_Fault_Detect(void);
static void VBAT_Fault_Detect(void);
static void IDC_Fault_Detect(void);
static void IPK_Fault_Detect(void);
static void PWR_Fault_Detect(void);
static void Robot_Fault_Code_Detect(void);
void Robot_Fault_TimeT1ms(void);
u16 Get_FltCode_Custm(void);

#endif
