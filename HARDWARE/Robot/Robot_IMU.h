#ifndef __ROBOT_IMU_H
#define __ROBOT_IMU_H
#include "sys.h"
#include "stm32f10x_i2c.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "Public.h"
#include "Robot_Command.h"

#define  	IMU_Addr 				0x68
#define  	IMU_ClockSpeed            	400000
#define 	q30  1073741824.0f


#define 	true 1
#define 	false 0 
#define 	TRUE  0
#define 	FALSE -1

//GPIOB.6 and 7
#define	I2C1_SCL_HI()		GPIOB->ODR |= (1<<6)     
#define	I2C1_SCL_LOW()		GPIOB->ODR &= ~(1<<6) 
#define	I2C1_SDA_HI()		GPIOB->ODR |= (1<<7)     
#define	I2C1_SDA_LOW()		GPIOB->ODR &= ~(1<<7) 
#define 	I2C1_SDA_READ()  	PBin(7)

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


static void I2C_delay(void);
static u8 I2C_Start(void);
static void I2C_Stop(void);
static void I2C_Ack(void);
static void I2C_NoAck(void);
static u8 I2C_WaitAck(void);
static void I2C_SendByte(u8 byte);
static u8 I2C_ReceiveByte(void);
u8 i2cWriteBuffer(u8 addr, u8 reg, u8 len, u8 * data);
int i2cwrite(u8 addr, u8 reg, u8 len, u8 * data);
u8 i2cWrite(u8 addr, u8 reg, u8 data);
int i2cread(u8 addr, u8 reg, u8 len, u8 *buf);
u8 i2cRead(u8 addr, u8 reg, u8 len, u8 *buf);
void Robot_I2C1_SW_Init(void);
void Robot_IMU_SW_Init(void);
s8 Robot_IMP_SW_DMP_Init(void);
void Robot_I2C1_HW_Init(void);
void Robot_IMU_Variable_Init(void);
u16 Robot_IMU_Out(u8 regist);
void IMU_Motion_SW_Read(void);
void Robot_IMU_Main(void);
void Robot_IMU_TimeT1ms(void);
void I2C1_EV_IRQHandler(void);


#endif

