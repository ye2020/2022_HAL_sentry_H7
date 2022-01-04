#include "pid.h"     
#include "maths.h"
#include "SysInit.h"
#include <math.h>

/*
*功能：对传入数据进行位置式pid运算并输出值
*传入：1.PID结构体名称   2.当前值
*传出：对该设备的控制量
*说明：目标值在闭环控制函数里赋值
*/
int32_t Location_Pid_Int32(PidTypeDef *pid , float actualValue) 
{
	//error
	pid->error[0] = pid->SetValue - actualValue;      
	
	//误差积分	
//	pid->Ierror += pid->error[0];
	pid->Ierror += (pid->error[0] + pid->error[1]) / 2;
	//积分限幅
	pid->Ierror = int32_limit(pid->Ierror, 10000, -10000);
	
	//微分项 记录
//	pid->Derror[2] = pid->Derror[1];
//	pid->Derror[1] = pid->Derror[0];
	pid->Derror[0] = (pid->error[0] - pid->error[1]);
	
	//输出pid运算
	pid->out = (pid->Kp * pid->error[0]) + (pid->Ki * pid->Ierror) + (pid->Kd * pid->Derror[0]);   
	
	//PID每一项输出
	pid->Pout = (pid->Kp * pid->error[0]);
	pid->Iout = (pid->Ki * pid->Ierror);
	pid->Dout = (pid->Kd * pid->Derror[0]);
	
	//记录error
	pid->error[1] = pid->error[0];   
	
	return pid->out;
}



fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)//不完整 (还没加模式和限制)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->SetValue = set;
	pid->ActualValue = ref;
	pid->error[0] = set - ref;

	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Derror[2] = pid->Derror[1];
	pid->Derror[1] = pid->Derror[0];
	pid->Derror[0] = (pid->error[0] - pid->error[1]);
	pid->Dout = pid->Kd * pid->Derror[0];
	pid->Iout = float_limit(pid->Iout, pid->max_iout, -pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
//	pid->out = float_limit(pid->out, pid->max_out, -pid->max_out);
	
	return pid->out;
}



//底盘pid初始化
void pid_init(PidTypeDef *pid, fp32 kp, fp32 ki, fp32 kd, fp32 i_max, fp32 out_max)
{
	if (pid == NULL)
	{
		return;
	}

	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

    pid->SetValue = 0;
    pid->ActualValue = 0;

    pid->out = 0;
	pid->lastout = 0;
	
	pid->Ierror = 0;
	
//	pid->max_iout = i_max;
//	pid->max_out = out_max;
	
	pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
	pid->error[0] = pid->error[0] = pid->error[0] = 0.0f;
	pid->Dout = pid->Iout = pid->Pout = pid->out = 0.0f;
}



void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Derror[0] = pid->Derror[1] = pid->Derror[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->ActualValue = pid->SetValue = 0.0f;
}



/*步进式PID控制设定值步进处理函数*/
float step_in_processing(PidTypeDef *vPID, float sp)
{
    //float stepIn = (vPID->maximum - vPID->minimum) * 0.1f + vPID->minimum;
	float stepIn = vPID->stepIn;
    float kFactor = 0.0f;

    if (fabs(vPID->SetValue - sp) <= stepIn)
    {
        vPID->SetValue = sp;
    }
    else
    {
        if (vPID->SetValue - sp > 0)
        {
            kFactor = -1.0f;
        }
        else if (vPID->SetValue - sp < 0)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }

        vPID->SetValue = vPID->SetValue + kFactor * stepIn;
    }

    return vPID->SetValue;
}



// NO_4 修改增加
int32_t pid_regulator(PidTypeDef *vPID , float actualValue)
{
	//error
	vPID->error[0] = vPID->SetValue - actualValue;      
	
	//误差积分	
//	if (BetaGeneration(vPID->error[0], 5.0f) == 1)
//	{
//		
//	}
	//vPID->Ierror += (vPID->error[0] + vPID->error[1]) / 2; //梯形积分
	
	//maximum和I  
	if (vPID->out > vPID->maximum) //vPID->maximum
	{
		if (vPID->error[0] <= 0)
		{
			vPID->Ierror += vPID->error[0];
		}
	}
	else if (vPID->out < vPID->minimum) //vPID->minimum
	{
		if (vPID->error[0] >= 0)
		{
			vPID->Ierror += vPID->error[0];
		}
	}
	else
	{
		vPID->Ierror += vPID->error[0];
	}
	
	//变积分
	//vPID->Ierror *= VariableIntegralCoefficient(vPID->error[0], vPID->errorabsmax, vPID->errorabsmin);
	
	//积分限幅
	vPID->Ierror = int32_limit(vPID->Ierror, vPID->max_iout, -vPID->max_iout);
	
	//微分项 记录
	//vPID->Derror[2] = vPID->Derror[1];
	//vPID->Derror[1] = vPID->Derror[0];
	vPID->Derror[0] = (vPID->error[0] - vPID->error[1]);
	
	//PID每一项输出
	vPID->Pout = (vPID->Kp * vPID->error[0]);
	vPID->Iout = (vPID->Ki * vPID->Ierror);
	vPID->Dout = (vPID->Kd * vPID->Derror[0]);
	//计算微分项增量带不完全微分
	//vPID->Dout = (vPID->Kd * vPID->Derror[0] * (1 - vPID->alpha)) + (vPID->alpha * vPID->Dout);
	
	//输出PID运算
	vPID->out = vPID->Pout + vPID->Iout + vPID->Dout;   
	
	//记录error
	vPID->error[1] = vPID->error[0];
	
	return vPID->out;
}


















