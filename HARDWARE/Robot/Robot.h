#ifndef  ROBOT_H
#define	ROBOT_H

#include "sys.h"
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
#include "Robot_IMU.h"
#include "Robot_ESP32.h"

/*Function declaration--------------------------------------------------------*/
extern void Robot_Init(void);
extern void Robot_Main(void);
extern void Robot_Action_TMR_1MS(void);

#endif
