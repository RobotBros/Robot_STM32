#ifndef __ROBOT_POWERON_H
#define __ROBOT_POWERON_H 

#include "sys.h"
#include "Robot_State.h"
#include "Robot_Fault.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

//GPIOA.2
#define	ROBOT_POWER_ON()		GPIOA->ODR |= (1<<2)     
#define	ROBOT_POWER_OFF()		GPIOA->ODR &= ~(1<<2) 


void Robot_Poweron_Init(void);
	 				    
#endif
