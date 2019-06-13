#ifndef __ROBOT_HEAD_LED_H
#define __ROBOT_HEAD_LED_H
#include "sys.h"
#include "stm32f10x_tim.h"

#define HEAD_PWM_DUTY_ARR				4499	  			//	PWM频率=72000/(4499+1)=16Khz / 2 = 8kHz(20ms)  
#define HEAD_PWM_DUTY_DIV					1		  			//	PWM频率DIV系数，微调DIV=1+1 
#define HEAD_MAX_DUTY						50					//	PWM Duty 50%

void TIM2_PWM_Init(u16 arr, u16 psc);
void Robot_Head_LED_PWM_Init(void);
void Robot_Head_LED_PWM_Percentage(u8 percentage);
 
#endif
