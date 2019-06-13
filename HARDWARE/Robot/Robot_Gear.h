#ifndef __ROBOT_GEAR_H
#define __ROBOT_GEAR_H	 
#include "sys.h"

/*define Gear TX frame--------------------------------------------*/
typedef struct
{
    u8   	Id; 		 			// Gear ID
    u8   	Command;			// Gear Command
    u8   	Led_flash;    	 	// uni	
    u8	Angle;    			// uni		
    u8      Action_times;		// uni
    u8      TX_done;			// Motor TX is done
    u8 	Sel_uni_comb;		// 0:uni ID, 1:16 motor

    u8	Id1_Angle;			// Set angle
    u8	Id2_Angle;
    u8	Id3_Angle;
    u8	Id4_Angle;
    u8	Id5_Angle;	
    u8	Id6_Angle;	
    u8	Id7_Angle;	
    u8	Id8_Angle;
    u8	Id9_Angle;
    u8	Id10_Angle;
    u8	Id11_Angle;	
    u8	Id12_Angle;	
    u8	Id13_Angle;	
    u8	Id14_Angle;	
    u8	Id15_Angle;	
    u8	Id16_Angle;		

    u8       Id1_Times;			// Set action times
    u8	Id2_Times;
    u8	Id3_Times;
    u8	Id4_Times;
    u8	Id5_Times;	
    u8	Id6_Times;	
    u8	Id7_Times;	
    u8	Id8_Times;
    u8	Id9_Times;
    u8	Id10_Times;
    u8	Id11_Times;	
    u8	Id12_Times;	
    u8	Id13_Times;	
    u8	Id14_Times;	
    u8	Id15_Times;	
    u8	Id16_Times;	
   
}tGear_TX_Frame;

typedef struct
{
    u8   	Id; 		 			// Gear ID
    u8	Command; 

    u8	Id1_Angle;			// read angle
    u8	Id2_Angle;
    u8	Id3_Angle;
    u8	Id4_Angle;
    u8	Id5_Angle;	
    u8	Id6_Angle;	
    u8	Id7_Angle;	
    u8	Id8_Angle;
    u8	Id9_Angle;
    u8	Id10_Angle;
    u8	Id11_Angle;	
    u8	Id12_Angle;	
    u8	Id13_Angle;	
    u8	Id14_Angle;	
    u8	Id15_Angle;	
    u8	Id16_Angle;	
	
    u8	Id1_AngleOffset;
    u8	Id2_AngleOffset;
    u8	Id3_AngleOffset;
    u8	Id4_AngleOffset;
    u8	Id5_AngleOffset;	
    u8	Id6_AngleOffset;	
    u8	Id7_AngleOffset;	
    u8	Id8_AngleOffset;
    u8	Id9_AngleOffset;
    u8	Id10_AngleOffset;
    u8	Id11_AngleOffset;	
    u8	Id12_AngleOffset;	
    u8	Id13_AngleOffset;	
    u8	Id14_AngleOffset;	
    u8	Id15_AngleOffset;	
    u8	Id16_AngleOffset;	
	
    u8	RX_done_flag;
}tGear_RX_Frame;

//------------Steering gear command frame
//TX frame:0xFA+0xAF+ID+command+parm1H+parm1L+parm2H+parm2L+checksum+0xED		Lenth 10 bytes
//TX checksum =  ID+command+parm1H+parm1L+parm2H+parm2L							Lenth  1 byte
//RX frame:0xFA+0xAF+ID+status+parm1H+parm1L+parm2H+parm2L+checksum+0xED			Lenth 10 bytes
//TX checksum =  ID+status+parm1H+parm1L+parm2H+parm2L								Lenth  1 byte
//-------------------------------------
#define ACTION_IDEL					0x00	
#define ACTION_TIME_COMMAND		0x01  	//0xFA+0XAF+ID+0x01+angle+TIME+timeH+timeL+checksum+0xED  ACK:0xAA+ID  TIME(20ms/1)
#define LED_COMMAND				0x04	//0xFA+0xAF+ID+0x04+LEDstate +0+0+0+0+checksum+0xED ACK:0xAA+ID
#define RD_ANGLE_COMMAND			0x02	//0xFA+0xAF+ID+0x02+0+0+0+0+checksum+0xED		      ACK:0xFA+0xAF+ID+0xXX+tagH+tagL+relH+relL+checksum+0xED
#define MODIFY_ID_COMMAND			0xCD	//0xFA+0xAF+ID+0xCD+0+ID(1-240)+0+0+checksum+0xED   ACK:0xFA+0xAF+ID+0xAA(EE)+0+ID+0+0+checksum+0xED
#define ANGLE_ADJ_COMMAND			0xD2	//0xFA+0xAF+ID+0xD2+FDH+FDL+BWH+BWL+checksum+0xED   ACK:0xFA+0xAF+ID+0xAA(EE)+0+0+0+0+checksum+0xED
#define RD_ADJ_COMMAND			0xD4	//0xFA+0xAF+ID+0xD4+0+0+0+0+checksum+0xED   		ACK:0xFA+0xAF+ID+0xAA(EE)+FDH+FDL+BWH+BWL+checksum+0xED

/*Gear check sum define------------------------------------------*/
#define ID1_CHECK_START			2
#define ID1_CHECK_END				8

/*Gear Read angle ID define---------------------------------------*/
#define MOTOR1						0
#define MOTOR2						1
#define MOTOR3						2
#define MOTOR4						3
#define MOTOR5						4
#define MOTOR6						5
#define MOTOR7						6
#define MOTOR8						7
#define MOTOR9						8
#define MOTOR10						9
#define MOTOR11						10
#define MOTOR12						11
#define MOTOR13						12
#define MOTOR14						13
#define MOTOR15						14
#define MOTOR16						15

#define TX_UNI						0
#define TX_COMB						1

#define ADD							0
#define MINUS						1

/*GEAR DIR ---GPIOB.2------------------------------------------*/
#define	GEAR_DIR_RX()		GPIOB->ODR |= (1<<2)     
#define	GEAR_DIR_TX()		GPIOB->ODR &= ~(1<<2) 

void Robot_Steering_Gear_Init(void);
void Robot_Uart3_Init(u32 bound);
void Robot_Usart3_DMA_Init(void);
void Robot_Gear_Var_Init(void);
s16 Robot_Gear_Angle_Out(u8 regist);
void Robot_Gear_Angle_Cla(u8 dir);
void Robot_Gear_Data_Input(tGear_TX_Frame pmotor);
void Robot_Gear_Nor_UniTX_Frame(void);
u8 Robot_Gear_Check_sum(u8 *buf, u8 start_byte_no, u8 end_byte_no);
void Robot_Gear_Nor_LED_TX(u8 id,u8 on_off);
void Robot_Gear_Nor_ID_TX(u8 id);
void Robot_Gear_Nor_Action_TX(u8 id,u8 angle,u8 time);
void Robot_Gear_Nor_ReadAngle_TX(u8 id);
void Robot_Gear_Action_TX(u8 id,u8 angle,u8 time);
void Robot_Gear_ReadAngle_TX(void);
void Robot_Gear_Init_TX_Handle(u8 timer);
void Robot_Gear_Nor_CombTX_Frame(void);
u8 Gear_RX_Check_Right(u8 *Adrr,u8 start_byte_no, u8 end_byte_no);
void Robot_Gear_Init_RX_Handle(void);
void Robot_Gear_Normal_RX_Handle(void);
void Robot_Gear_TX_Test(void);
void Robot_Steering_Gear_Main(void);
static void UartGear_RXD_IRQ(void);
void Robot_Steering_Gear_TimeT1ms(void);
void USART3_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);

#endif

