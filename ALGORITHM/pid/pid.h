#ifndef _PID_H
#define _PID_H
#include "main.h"


typedef struct //pid结构体变量
{
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 SetValue;
    fp32 ActualValue;

		fp32 stepIn;

	    /* 变积分 */
    fp32 errorabsmax; //偏差绝对值最大值
    fp32 errorabsmin; //偏差绝对值最小值

	
		    /* 抗积分饱和 */
    fp32 maximum; //最大值
    fp32 minimum; //最小值
	
    fp32 out;
	fp32 lastout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
	
	fp32 Ierror;
    fp32 Derror[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3];   //误差项 0最新 1上一次 2上上次
} PidTypeDef;


extern int32_t Location_Pid_Int32(PidTypeDef *pid , float actualValue);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void pid_init(PidTypeDef *pid,fp32 kp,fp32 ki,fp32 kd,fp32 i_max,fp32 out_max);
extern void PID_clear(PidTypeDef *pid);
float step_in_processing(PidTypeDef *vPID, float sp);
int32_t pid_regulator(PidTypeDef *vPID , float actualValue);


#endif















