#ifndef __PID_H__
#define __PID_H__
#include "main.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float actual_value;			//ʵ��ֵ
	float Target_value;			//����ֵ
	float Err_value;					//ƫ��
	float Err_Last_value;		//�ϴ�ƫ��
	float Err_Sum;						//�ۼ�ƫ��
	float umax;
	
}PIDstructure;

void PID_Init(void);
float P_realize(PIDstructure *pid,float actual_val);
float PI_realize(PIDstructure *pid,float actual_val);
float PID_realize(PIDstructure *pid,float actual_val);


#endif
