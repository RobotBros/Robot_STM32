/*******************************************************************************
 * FileName£ºRobot_IPK_Fault.c
 * Function: Robot Input Current Peak Fault Handle function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_IPK_Fault.h"

/*static variable define-----------------------------------------------------------*/ 
static u8 IPK_state = 0;

/******************************************************************************
 * Routine     : Robot_IPK_Fault_Init
 * Function    : Robot IPK Fault I/O Initialization GPIOC.2
 ******************************************************************************/
void Robot_IPK_Fault_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);											
	
/*GPIOC.2 IPK Fault I/O-----------------------------------------------------*/		
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource2);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line2;																		
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	

	NVIC_SetPriority(EXTI2_IRQn,0);							//IRQ0										
	NVIC_EnableIRQ(EXTI2_IRQn);
	
	//NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;														
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;								
  	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;												
  	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;															
  	//NVIC_Init(&NVIC_InitStructure);  	  		

	IPK_state = 0;
}

/*******************************************************************************
 * Routine   : Robot_Hardware_Fault_Out
 * Function  : Register Read From Robot IPK Fault
 ******************************************************************************/
u8 Robot_Hardware_Fault_Out(void)
{  
	return IPK_state;
}

/******************************************************************************
 * Routine     : EXTI2_IRQHandler
 * Function    : Robot IPK Fault Interrupt Handle
 ******************************************************************************/
void EXTI2_IRQHandler(void)
{	
	if(EXTI_GetFlagStatus(EXTI_Line2) != RESET)
	{
		IPK_state = 1;
		ROBOT_POWER_OFF();						// Turn OFF Gear Power

		EXTI_ClearITPendingBit(EXTI_Line2);  							 						
	}
}

