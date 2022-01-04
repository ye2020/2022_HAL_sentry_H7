#ifndef __MATHS_H
#define __MATHS_H


#include "main.h"




//运动加速度限制斜坡函数
typedef __packed struct
{
    float Input;      //当前取样值
    float Last_Input; //上次取样值
    float Output;     //输出值
    float acc_now;    //当前加速度
    float acc_limit;  //需要限制的加速度
} acceleration_control_type_t;

//滑动均值滤波参数（浮点）
typedef __packed struct
{
    float Input;     //当前取样值
    int count_num;   //取样次数
    float Output;    //滤波输出
    float Sum;       //累计总和
    float FIFO[250]; //队列
    int sum_flag;    //已经够250个标志
} sliding_mean_filter_type_t;

//一阶低通滤波参数
typedef __packed struct
{
	fp32 input;        //输入数据
	fp32 last_input;   //上次数据
	fp32 out;          //滤波输出的数据
	fp32 num;          //滤波参数
}first_order_filter_type_t;



int32_t int32_limit(int32_t x, int32_t max, int32_t min);
int16_t int16_limit(int16_t x,int16_t max,int16_t min);
float float_limit(float x,float max,float min);
signed long limit_long(signed long x,signed long max,signed long min);
int16_t int16_t_abs(int16_t x);
signed long long_abs(signed long x);
float float_abs(float x);			// 绝对值计算
float invSqrt(float x);       //平方根倒数

/* 运动控制斜坡函数（加速度限制）（16位） */
int16_t Motion_acceleration_control(acceleration_control_type_t *acceleration_control, int16_t Input, int16_t Limit); //运动加速度限制

/* 低通滤波 */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* 平滑滤波 */
float Sliding_Mean_Filter(sliding_mean_filter_type_t *mean_filter, float Input, int num);  //均值滑窗滤波
void Sliding_Mean_Filter_Init(sliding_mean_filter_type_t *mean_filter);  //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）

/* 循环限制 */
int16_t Loop_Restriction_Int16(int16_t num, int16_t limit_num);          //16位循环限幅
float Loop_Restriction_Float(float num, float limit_num);                //浮点循环限幅
float loop_fp32_constrain(float Input, float minValue, float maxValue);  //循环限制（云台角度处理）
float cos_calculate(float angle);
float sin_calculate(float angle);

/* 斜坡函数 */
void Data_Accelerated_Control(float *input, float acc);                                                               //加速度限制斜坡函数

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif

