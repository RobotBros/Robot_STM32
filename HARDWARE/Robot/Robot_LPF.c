/*******************************************************************************
 * FileName：Robot_LPF.c
 * Function:   Robot Low pass filter
 * Designer:  Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
 
/*低通滤波器：
一阶惯性环节的数学表达式为：T*dc(t)/dt +c(t) =r(t),取拉式变换，有TsC(s)+C(s) =R(s),C(s)=R(s)/(Ts +1),则一阶RC低通滤波器的传递函数为：
G(s) =C(s)/R(s) =1/(Ts +1)
式中：C(t)为输出变量，r(t)为输入变量，T为时间常数。
一、一阶低通滤波传递函数(RC阻容滤波)：
(1) H(jw)=1/(1+jWRC) 
(2) Wc = 1/RC (截止频率) Wc = 2*pi*fc 
    H(jw) = 1/ SQR(1+(W/Wc)^2)
    W<Wc 幅值不衰减，相位0
    W>Wc 幅值一直衰减 相位-90
    W=Wc 幅值衰减至0.707Vin 相位-45
二、数字离散化：Z变换差分法分为向前差分法和向后差分法，但是由于向前差分法稳定性较差，故一般采用稳定性较好的向后差分法。
(1) H(s) = 1/(RCs+1) 
(2) s={1-z(-1)}/T T:采样周期
(3) Y(z)/X(z) = T/(T+RC)*X(z)+RC/(T+RC)*Y[z]*z(-1)  ==> X(z)=(1+RC/T)*Y(z)-RC/T*Y(z)*Z(-1) => X(n)=(1+RC/T)*Y(n)-RC/T*Y(n-1)
(4) Y[n]/X[n] = T/(T+RC)*X(n)+RC/(T+RC)*Y[n-1]
三、差分方程为：
(1) Y[n] = Kc*X[n] +(1-Kc)Y[n-1] = Kc*(X[n]-Y[n-1])+Y[n-1]
(2) Kc = T/(T+RC) = 1/(1+τ/T) τ:时间常数
(3) Wc = 1/RC  ==> Kc = T/(T+1/Wc) = Wc/(Wc+f)  
式中：Xn   ：本次采样值
      Yn-1 ：上次的滤波输出值
     a(Kc) ：滤波系数，其值通常远小于1
	Yn ：本次滤波的输出值
	f  : 采样频率
*/

#include "Robot_LPF.h"

/***************************************************************************
 * Routine     : 低通滤波（移位）适用于固定滤波带宽精度不高的场合
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)={x(n)-y(n-1)}>>N/s
                 T = (2^N -1)*TsLPF    2^N=f/Wc+1
 * Input       : Xn,N
 * Run Time    : 30 cycles (Not Optimized 33 cycles)
 ***************************************************************************/
s16 LPFilt_2N(s16 In_data, tFilt *Lpf)
{
	s16 tmp16;
	
	if(Lpf->Enable) 
	{
		tmp16  = In_data - Lpf->YnLst;  // x(n) - y(n-1)
		Lpf->IntSum = Lpf->IntSum + tmp16 ;
		Lpf->Yn  = Lpf->IntSum >> Lpf->N;
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return (Lpf->Yn);
}

/***************************************************************************
 * Routine     : 低通滤波，适用于固定滤波带宽的场合
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)
                 K = 2^15*Wc/(Wc+f)  k = 0~1
 * Input       : Xn,K
 * Run Time    : 31 cycles (Not Optimized 36 cycles)
 ***************************************************************************/
s16 LPFilt_WcTs(s16 In_data, tFilt *Lpf)
{
	s32 tmp32;
	s16 tmp16;

	if(Lpf->Enable)
	{
		tmp16 = In_data-Lpf->YnLst;
		Lpf->IntSum =Lpf->IntSum +tmp16;
		tmp32 =Lpf->IntSum*Lpf->K;
		Lpf->Yn =(s16)(tmp32>>15);
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return(Lpf->Yn);
}

/***************************************************************************
 * Routine     : 低通滤波，适用于变化滤波带宽的场合
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)
                 K = Wc/(Wc+f)   f=Hz
 * Input       : Xn,Wc,FHz
 * Run Time    : 53 cycles (Not Optimized 59 cycles)
 ***************************************************************************/
s16 LPFilt_Wc(s16 In_data, tFilt *Lpf)
{
	s32 tmp32;
	s16 tmp16;
	
	if(Lpf->Enable)
	{	 
		tmp16 = In_data - Lpf->YnLst;  
		Lpf->IntSum = Lpf->IntSum + tmp16 ;
		tmp32 = Lpf->Wn;
		tmp32 = tmp32<<15;
		tmp16 = Lpf->Wn + Lpf->FHz;
		tmp32 = tmp32/tmp16;
		tmp32 = tmp32*Lpf->IntSum;
		Lpf->Yn = tmp32>>15;
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return (Lpf->Yn);
}
