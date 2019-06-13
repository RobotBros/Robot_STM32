#ifndef __ROBOT_SAMPLE_H
#define __ROBOT_SAMPLE_H	
#include "sys.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dma.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

#define ADC1_VBAT_BYTE		0
#define ADC1_IBUS_BYTE		1

#define FILITER_DISABLE		0
#define FILITER_ENABLE		1

#define VBAT_OVER_VOLTAGE  	2638	//8.5V
#define VBAT_UNDER_VOLTAGE	2234 	//7.2V
#define VBAT_OVER_CURRENT		838		//15A		

void Robot_Sample_Init(void);
void  Robot_Adc_Init(void);
void Robot_Adc_DMA_Init(void);
u16 Robot_Sample_Out(u8 regist, u8 fliter_en);
 void Robot_Sample_Main(void);
 void Robot_Sample_TimeT1ms(void);
 /*DMA1 ADC1 Interrupt-----------------------------*/
 void DMA1_Channel1_IRQHandler(void);

#endif 
