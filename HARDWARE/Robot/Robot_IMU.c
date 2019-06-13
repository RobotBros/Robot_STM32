/*******************************************************************************
 * FileName：Robot_IMU.c
 * Function: Robot IMU MPU6050 drive function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	    
#include "Robot_IMU.h"

/*static variable define-----------------------------------------------------------*/ 
static float *ImuReg_Addr[3];					// IMU register witch filter 0:Pitch 1:Roll 2:Yaw
static float Pitch = 0.0f;						
static float Roll = 0.0f;
static float Yaw = 0.0f;

static u8 T100msFlag_IMU = 0;
static s8 gyro_orientation[9] =	      	      {1,   0,  0,		//X						|
                                          			 0,   1,  0,		//Y						| 
                                          			 0,   0,  1};	   	//Z 定义航向正方向   Y<----Z    Pitch x轴旋转；Roll Y轴；Yaw Z轴 
                                          			 
//--------------------------IO Simulation I2C------------------------------------//
/*******************************************************************************
 * Routine: I2C_delay
 * Function: I2C1 delay
 ******************************************************************************/
static void I2C_delay(void)
{
    volatile u8 cnt = 7;
	
    while (cnt)
        cnt--;
}

/*******************************************************************************
 * Routine: I2C_Start
 * Function: I2C1 Start IO state
 ******************************************************************************/
static u8 I2C_Start(void)
{
	I2C1_SDA_HI();
    	I2C1_SCL_HI();
    	I2C_delay();
    	if (!I2C1_SDA_READ())	return false;
	I2C1_SDA_LOW();
    	I2C_delay();
   	if (I2C1_SDA_READ())		return false;
	I2C1_SDA_LOW();
    	I2C_delay();
    	return true;
}

/*******************************************************************************
 * Routine: I2C_Stop
 * Function: I2C1 Stop IO state
 ******************************************************************************/
static void I2C_Stop(void)
{
	I2C1_SCL_LOW();
    	I2C_delay();
	I2C1_SDA_LOW();
    	I2C_delay();
	I2C1_SCL_HI();	
    	I2C_delay();
	I2C1_SDA_HI();	
    	I2C_delay();
}

/*******************************************************************************
 * Routine: I2C_Ack
 * Function: I2C1 ACK IO state
 ******************************************************************************/
static void I2C_Ack(void)
{
	I2C1_SCL_LOW();
    	I2C_delay();
	I2C1_SDA_LOW();	
    	I2C_delay();
	I2C1_SCL_HI();		
    	I2C_delay();
	I2C1_SCL_LOW();	
    	I2C_delay();
}

/*******************************************************************************
 * Routine: I2C_NoAck
 * Function: I2C1 No ACK IO state
 ******************************************************************************/
static void I2C_NoAck(void)
{
	I2C1_SCL_LOW();
    	I2C_delay();
	I2C1_SDA_HI();		
    	I2C_delay();
	I2C1_SCL_HI();	
    	I2C_delay();
	I2C1_SCL_LOW();
    	I2C_delay();
}

/*******************************************************************************
 * Routine: I2C_WaitAck
 * Function: I2C1 Wait ACK IO state
 ******************************************************************************/
static u8 I2C_WaitAck(void)
{
	I2C1_SCL_LOW();
   	I2C_delay();
	I2C1_SDA_HI();
    	I2C_delay();
	I2C1_SCL_HI();	
    	I2C_delay();
    	if (I2C1_SDA_READ()) 
	{
       	I2C1_SCL_LOW();
        	return false;
    	}
    	I2C1_SCL_LOW();
    	return true;
}

/*******************************************************************************
 * Routine: I2C_SendByte
 * Function: I2C1 Send 1 byte IO state
 ******************************************************************************/
static void I2C_SendByte(u8 byte)
{
	u8 cnt8 = 8;
	
    	while (cnt8--) 
	{
		I2C1_SCL_LOW();
        	I2C_delay();
        	if (byte & 0x80)	I2C1_SDA_HI();
        	else		I2C1_SDA_LOW();
        	byte <<= 1;
        	I2C_delay();
		I2C1_SCL_HI();	
        	I2C_delay();
    	}
	I2C1_SCL_LOW();	
}

/*******************************************************************************
 * Routine: I2C_ReceiveByte
 * Function: I2C1 Receive 1 byte IO state
 ******************************************************************************/
static u8 I2C_ReceiveByte(void)
{
	u8 cnt8 = 8;
    	u8 byte = 0;

	I2C1_SDA_HI();

    	while (cnt8--)
	{
        	byte <<= 1;
		I2C1_SCL_LOW();	
        	I2C_delay();
		I2C1_SCL_HI();	
        	I2C_delay();
        	if (I2C1_SDA_READ())
		{
            		byte |= 0x01;
       	}
    	}
	I2C1_SCL_LOW();	
    	return byte;
}

/*******************************************************************************
 * Routine: i2cWriteBuffer
 * Function: I2C1 Write x bytes IO state
 ******************************************************************************/
u8 i2cWriteBuffer(u8 addr, u8 reg, u8 len, u8 * data)
{
   u8 cnt8 = 0;
	
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (cnt8 = 0; cnt8 < len; cnt8++) {
        I2C_SendByte(data[cnt8]);
        if (!I2C_WaitAck()) {
            I2C_Stop();
            return false;
        }
    }
    I2C_Stop();
    return true;
}

/*******************************************************************************
 * Routine: i2cwrite
 * Function: I2C1 Write x bytes IO state 
 ******************************************************************************/
int i2cwrite(u8 addr, u8 reg, u8 len, u8 * data)
{
	if(i2cWriteBuffer(addr,reg,len,data))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}

/*******************************************************************************
 * Routine: i2cwrite
 * Function: I2C1 Write 1 bytes include address IO state 
 ******************************************************************************/
u8 i2cWrite(u8 addr, u8 reg, u8 data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}

/*******************************************************************************
 * Routine: i2cread
 * Function: I2C1 read x bytes IO state 
 ******************************************************************************/
int i2cread(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(i2cRead(addr,reg,len,buf))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	//return FALSE;
}

/*******************************************************************************
 * Routine: i2cRead
 * Function: I2C1 Read 1 bytes include address IO state 
 ******************************************************************************/
u8 i2cRead(u8 addr, u8 reg, u8 len, u8 *buf)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}

/*******************************************************************************
 * Routine: Robot_I2C1_SW_Init
 * Function: Robot I2C1 IO Port Software Simulation Initialization
 * Remark : IO simulation I2C 
 ******************************************************************************/
void Robot_I2C1_SW_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	I2C1_SCL_HI();
	I2C1_SDA_HI();
}

/*******************************************************************************
 * Routine: Robot_IMU_SW_Init
 * Function: IMU IO simulation I2C Initialization
 ******************************************************************************/
void Robot_IMU_SW_Init(void)
{
	Robot_IMU_Variable_Init();				// Initial Variable
	
	Robot_I2C1_SW_Init();					// Initial I2C1 IO
	
	if(Robot_IMP_SW_DMP_Init() == TRUE)		// Initial IMU DMP
	{
		Robot_Command_IMU_Enable(ON);
	}
	else
	{
		Robot_Command_IMU_Enable(OFF);
	}
}

/*******************************************************************************
 * Routine: Robot_IMP_SW_DMP_DET
 * Function: IMU Software IO simulation I2C DMP Initialization
 ******************************************************************************/
s8 Robot_IMP_SW_DMP_Init(void)
{
	if(!mpu_init())
	{			
/*mpu_set_sensor--------------------------------------------------------*/	
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));			
		else return FALSE;
/*mpu_configure_fifo------------------------------------------------------*/			
	  	if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL));	   
	  	else return FALSE;
/*mpu_set_sample_rate----------------------------------------------------*/	
	  	if(!mpu_set_sample_rate(DEFAULT_MPU_HZ));	   	 			 
	  	else return FALSE;
/*dmp_load_motion_driver_firmvare------------------------------------------*/
		if(!dmp_load_motion_driver_firmware());  	 			
	 	else return FALSE;
/*dmp_set_orientation-----------------------------------------------------*/			
		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))); 	  
	 	 else return FALSE;
/*dmp_enable_feature-----------------------------------------------------*/	
		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        					     DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        					     DMP_FEATURE_GYRO_CAL));		   	  
		else return FALSE;
/*dmp_set_fifo_rate-----------------------------------------------------*/			
		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ));  	  				
	  	else return FALSE;
/*dmp_run_self_test-----------------------------------------------------*/			
	  	run_self_test();
/*dmp_mpu_set_dmp_state enable-----------------------------------------*/		
	 	if(!mpu_set_dmp_state(ON)) 	return TRUE;
		else return FALSE;
	}
	else return FALSE;
}

/*******************************************************************************
 * Routine: Robot_I2C1_HW_Init
 * Function: IMU Hardware I2C1 Initialization
 ******************************************************************************/
void Robot_I2C1_HW_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

/*Initial I2C1 DMA Funciton------------------------------------------------------*/
	//Robot_I2c1_DMA_Init();
/*Initial I2C1 Clock------------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);										
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);							
	I2C_DeInit(I2C1); 															
	
/*I2C1 I/O SCL---------------------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;															
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 
   
/*I2C1 I/O SDA---------------------------------------------------------------*/
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;															
  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

/* I2C1  configuration---------------------------------------------------------- */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = IMU_Addr;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = IMU_ClockSpeed;
    I2C_Init(I2C1, &I2C_InitStructure);
/* I2C1 IRQ----------------------------------------------------------------- */
    NVIC_SetPriority(I2C1_EV_IRQn, 6); 					// I2C1 Event IRQ6
    NVIC_EnableIRQ(I2C1_EV_IRQn);	
}



/*******************************************************************************
 * Routine     : Robot_IMU_Variable_Init
 * Function    : Robot IMU Variable Initialization
 ******************************************************************************/
void Robot_IMU_Variable_Init(void)
{
/*variable initial----------------------------------------------------------------*/ 	
	ImuReg_Addr[0] = (float*)&Pitch;
	ImuReg_Addr[1] = (float*)&Roll;		
	ImuReg_Addr[2] = (float*)&Yaw;	
}

/*******************************************************************************
 * Routine   : Robot_IMU_Out
 * Function  : Read IMU Value 
 * Remark   : Return IMU Value:regist:Pitch,Roll,Yaw
 ******************************************************************************/
u16 Robot_IMU_Out(u8 regist)
{  
	if(regist < 4)	return (*ImuReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine: IMU_Motion_SW_Read
 * Function: IMU Motion Software Read Four Elememts
 ******************************************************************************/
void IMU_Motion_SW_Read(void)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];

	if(Robot_Command_Out(ROBOT_IMU_ENABLE))
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);   					// read DMP FIFO data

		if (sensors & INV_WXYZ_QUAT )
		{
			q0=quat[0] / q30;	
			q1=quat[1] / q30;
			q2=quat[2] / q30;
			q3=quat[3] / q30;
			Pitch  =  asin(2 * q1 * q3 - 2 * q0* q2)* 57.3*10; 							   		// pitch*10
    	 		Roll   =   atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3*10; 		// roll*10
			Yaw   = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3*10;		   		// yaw*10
		}
	}
}

/*******************************************************************************
 * Routine: Robot_IMU_Main
 * Function: IMU Main Function
 ******************************************************************************/
void Robot_IMU_Main(void)
{
	if(T100msFlag_IMU)
	{
		T100msFlag_IMU = 0;
		IMU_Motion_SW_Read();
		// Gear action handle
	}
}

/*******************************************************************************
 * Routine: Robot_IMU_TimeT1ms
 * Function: IMU Function Timebase 100ms
 ******************************************************************************/
void Robot_IMU_TimeT1ms(void)
{
    static u8 T1msCnt=0;

/*100ms-------------------------------------------------------------------*/
	if(++T1msCnt>=100)
	{
		T1msCnt = 0;
    	T100msFlag_IMU = 1;
	}
}


/*******************************************************************************
 * Routine: I2C1_EV_IRQHandler
 * Function: I2C1 Event IRQ Handler
 ******************************************************************************/
void I2C1_EV_IRQHandler(void)
{

}

