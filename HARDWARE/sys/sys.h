#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include "Public.h"
#include "Robot.h"
#include "Customer.h"
#include "Robot_Command.h"
#include "Robot_Fault.h"
#include "Robot_Flash.h"
#include "Robot_Fault.h"
#include "Robot_Gear.h"
#include "Robot_Head_LED.h"
#include "Robot_IPK_Fault.h"
#include "Robot_LED.h"
#include "Robot_LPF.h"
#include "Robot_Poweron.h"
#include "Robot_Sample.h"
#include "Robot_State.h"
//#include "Robot_ESP32.h"
#include "Robot_Sound.h"

#define SYS_TICK_1MS					8999 		// 72M/9000/8 = 1ms

#define	ON								1
#define	OFF								0

/*Bit operation define--------------------------------------------------------*/
#define	BitSet(REG,NUM)  ((REG) |= (1UL<<(NUM)))
#define	BitClr(REG,NUM)  ((REG) &= (~(1UL<<(NUM))))
#define	BitXor(REG,NUM)  ((REG) ^= (1UL<<(NUM)))
#define	BitVal(REG,NUM)  ((REG)&(1UL<<(NUM)))
	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/*Function  define------------------------------------------------------------*/
#define Disable_IRQ()  __disable_irq() 						 //Disable All Interrupt
#define Enable_IRQ()   __enable_irq()  						 //Enable All Interrupt
//#define Enable_WDT()   			//Enable watchdog timer  
//#define Disable_WDT()   //Disable watchdog timer 
//#define Clear_WDT()   //Clear watchdog timer 
//#define Fault_WDT()   //write Err magic word to Reset MCU 
//#define SW_Reset()	//SoftWare to reset MCU

void System_Init(void);

#endif

