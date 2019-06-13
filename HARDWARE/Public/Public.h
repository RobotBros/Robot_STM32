#ifndef PUBLIC_H
#define PUBLIC_H

#include "Sys.h"
#include "Customer.h"
#include "Robot.h"

/*Function  define------------------------------------------------------------*/
//test signal
#define	TEST_PIN_INV()		GPIOC->ODR ^= (1<<4) 
#define	TEST_PIN_LOW()		GPIOC->ODR &= ~(1<<4)     

/*Function  Declaration-------------------------------------------------------*/
void Public_Init(void);
void NVIC_Configuration(void);
static void SysTick_Init(void);
void Robot_Test_PIN_Init(void);
void SysTick_Handler(void);
void Delay(u16 delay_times);
void delay_ms(u16 nms);

#endif
