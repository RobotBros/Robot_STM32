#ifndef ROBOT_LPF_H
#define	ROBOT_LPF_H

#include "Sys.h"

typedef struct
{
    s16     	Enable; 		// Input:Filter OnOFF Switch (0~1)
    s16     	Wn;     		// Input:Filter bandwidth (0~32767)
    s16		FHz;    		// 滤波采样频率
    s16		K;      		// 滤波系数
    s16   		N;      		// 滤波移位系数
    s16     	YnLst; 		// Inner variable (-32768~32767)
    s16     	Yn; 			// Inner variable (-32768~32767)
    s32     	IntSum; 	// Inner variable
}tFilt;

s16 LPFilt_2N(s16 In_data, tFilt *Lpf);
s16 LPFilt_WcTs(s16 In_data, tFilt *Lpf);
s16 LPFilt_Wc(s16 In_data, tFilt *Lpf);

#endif
