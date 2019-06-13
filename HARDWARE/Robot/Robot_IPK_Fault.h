#ifndef __ROBOT_IPK_FAULT_H
#define __ROBOT_IPK_FAULT_H	 

#include "sys.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "Robot_Poweron.h"
	 
void Robot_IPK_Fault_Init(void);
u8 Robot_Hardware_Fault_Out(void);
void EXTI2_IRQHandler(void);

#endif
