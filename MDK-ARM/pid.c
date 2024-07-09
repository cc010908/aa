#include "PID.H"

extern int16_t goal_speed ;

PIDstructure PIDMotorSpeed1;
PIDstructure PIDMotorSpeed2;
PIDstructure OpenMV_Pixel;
PIDstructure Turnspeed;

void PID_Init(void)
{
	PIDMotorSpeed1.actual_value   = 0;
	PIDMotorSpeed1.Target_value   = 2.0;
	PIDMotorSpeed1.Err_value      = 0;
	PIDMotorSpeed1.Err_Last_value = 0;
	PIDMotorSpeed1.Err_Sum        = 0;
	PIDMotorSpeed1.Kp             = 30;
	PIDMotorSpeed1.Ki             = 5;
	PIDMotorSpeed1.Kd             = 0;
	PIDMotorSpeed1.umax           = 100;
	
	PIDMotorSpeed2.actual_value   = 0;
	PIDMotorSpeed2.Target_value   = 2.0;
	PIDMotorSpeed2.Err_value      = 0;
	PIDMotorSpeed2.Err_Last_value = 0;
	PIDMotorSpeed2.Err_Sum        = 0;
	PIDMotorSpeed2.Kp             = 30;
	PIDMotorSpeed2.Ki             = 5;
	PIDMotorSpeed2.Kd             = 0;	
	PIDMotorSpeed2.umax           = 100;
	
	OpenMV_Pixel.actual_value   = 0;
	OpenMV_Pixel.Target_value   = 1200;
	OpenMV_Pixel.Err_value      = 0;
	OpenMV_Pixel.Err_Last_value = 0;
	OpenMV_Pixel.Err_Sum        = 0;
	OpenMV_Pixel.Kp             = 20;
	OpenMV_Pixel.Ki             = 0.2;
	OpenMV_Pixel.Kd             = 0;
	OpenMV_Pixel.umax           = 100;
	
	Turnspeed.actual_value   = 0;
	Turnspeed.Target_value   = 0;
	Turnspeed.Err_value      = 0;
	Turnspeed.Err_Last_value = 0;
	Turnspeed.Err_Sum        = 0;
	Turnspeed.Kp             = 10;
	Turnspeed.Ki             = 0.9;
	Turnspeed.Kd             = 0;
	Turnspeed.umax           = 50;	
}
//P调节
float P_realize(PIDstructure *pid,float actual_val)
{
	pid->actual_value = actual_val;
	pid->Err_value = pid->Target_value - pid->actual_value;
	pid->actual_value = pid->Kp*pid->Err_value;

	return pid->actual_value;
}
//PI调节
float PI_realize(PIDstructure *pid,float actual_val)
{
	pid->actual_value = actual_val;
	pid->Err_value = pid->Target_value - pid->actual_value;
	pid->Err_Sum += pid->Err_value;
	
	
	pid->actual_value = pid->Kp*pid->Err_value + pid->Ki*pid->Err_Sum;
	
	return pid->actual_value;
}
//PID调节
float PID_realize(PIDstructure *pid,float actual_val)
{
	pid->actual_value = actual_val;
	pid->Err_value = pid->Target_value - pid->actual_value;
	pid->Err_Sum += pid->Err_value ;
	
		if(pid->Err_Sum>pid->umax)
	{
	 pid->Err_Sum=pid->umax;
	}
	else if(pid->Err_Sum<-pid->umax)
	{
	 pid->Err_Sum=-pid->umax;
	}
	
	pid->actual_value = pid->Kp*pid->Err_value + pid->Ki*pid->Err_Sum + pid->Kd*(pid->Err_value - pid->Err_Last_value);
	
	pid->Err_Last_value = pid->Err_value;
	
	return pid->actual_value;
}

