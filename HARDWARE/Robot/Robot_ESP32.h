#ifndef  __ROBOT_ESP32_H
#define  __ROBOT_ESP32_H

#include "sys.h"
#include "Robot_Gear.h"
#include "string.h"
#include "Robot_Sound.h"

#define ESP_TX_SIZE		21
#define ESP_RX_SIZE		20

#define	ESP32_ON()		GPIOA->ODR |= (1<<9)     
#define	ESP32_OFF()		GPIOA->ODR &= ~(1<<9) 

#define	ESP32_INT_DIS()		GPIOB->ODR |= (1<<5)     
#define	ESP32_INT_ACT()		GPIOB->ODR &= ~(1<<5) 

void Robot_ESP32_Init(void);
void Robot_ESP32_SPI2_Init(void);
void Robot_ESP32_SPI2_DMA_Init(void);
u8 RX_Check_Right(u16 Size, u8 *Adrr);
static void RXD_Data_ESP32(void);
static void TXD_Data_ESP32(void);
void Robot_ESP32_Commu_Main(void);
void DMA1_Channel5_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void Robot_ESP32Com_TimeT1ms(void);
void ESP32_IRQHandler(void);
void EXTI9_5_IRQHandler(void);

#endif
