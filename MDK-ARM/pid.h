#ifndef __PID_H__
#define __PID_H__
#include "main.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float actual_value;			//实际值
	float Target_value;			//期望值
	float Err_value;					//偏差
	float Err_Last_value;		//上次偏差
	float Err_Sum;						//累计偏差
	float umax;
	
}PIDstructure;

void PID_Init(void);
float P_realize(PIDstructure *pid,float actual_val);
float PI_realize(PIDstructure *pid,float actual_val);
float PID_realize(PIDstructure *pid,float actual_val);


#endif
