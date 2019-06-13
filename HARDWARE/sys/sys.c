/*******************************************************************************
 * FileName£ºSys.c
 * Function: System Initialization function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "sys.h"

/*******************************************************************************
 * Routine   : System_Init
 * Function  : System Initialization
 ******************************************************************************/
void System_Init(void)
{
	//u16 cnt16 = 0;
	
	/* MCU clock is set at startup_stm32f10x_hd.s file Reset_Handler before main() SystemInit();*/
	/* Disable All interrupt--------------------------------------------------*/
    	Disable_IRQ(); 
	
	/* Disable Watchdog ---------------------------------------------------*/
	//Disable_WDT(); 	    		
	
	/* Public Initialization--------------------------------------------------*/
	Public_Init();

	/* Robot Initialization-----------------------------------------------------*/
	Robot_Init();
	
	/* Customer Initialization of Customer -----------------------------------*/
	Customer_Init();

	//for(cnt16=0;cnt16<5000;cnt16++)	{Delay(5000);}
	
	/* Enable Watchdog ---------------------------------------------------*/
	//Enable_WDT();
	
	/* Enable All interrupt--------------------------------------------------*/
	Enable_IRQ();						
}

