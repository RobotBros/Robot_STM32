#ifndef ROBOT_LED_H
#define	ROBOT_LED_H

#include "sys.h"
#include "Robot_State.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

//GPIOC.13
#define	LED_OFF()		GPIOC->ODR |= (1<<13)     
#define	LED_ON()		GPIOC->ODR &= ~(1<<13) 
#define	LED_INV()		GPIOC->ODR ^= (1<<13) 

void Robot_LED_Init(void);
void Robot_LED_Main(void);
static void LED_IDLE(void);
static void LED_RUN(void);
static void LED_FLT(void);
static void LED_DEBUG(void);
static void LED_FLT_Code(u8 FltCode);
void Robot_LED_TimeT1ms(void);

#endif
