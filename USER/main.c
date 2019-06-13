#include "sys.h"

void main(void)
{	
	System_Init();

  	while(1)
	{	
		/* Robot Modify ID Function-------------------------------------------------*/
		//Robot_Gear_TX_Test();
	
		/* Robot Main Function-----------------------------------------------------*/
		Robot_Main();	

		/* Customer Main Function--------------------------------------------------*/
		Customer_Main();

		/* Clear Watchdog--------------------------------------------------------*/
		//Clear_WDT();
	}	 
}

