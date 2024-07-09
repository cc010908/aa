#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

void Motor_Set(int Motor1,int Motor2)
{

	
	//1.�ȸ����������÷���GPIO �ߵ͵�ƽ
	if(Motor1 <0) 
	{AIN1_SET;AIN2_RESET;}
	else
	{AIN1_RESET;AIN2_SET;}//ǰ��
	
	if(Motor2 <0) 
	{BIN1_SET;BIN2_RESET;}	
	else  
	{BIN1_RESET;BIN2_SET;}//ǰ��
	//2.Ȼ������ռ�ձ�  
	if(Motor1 <0)
	{
		if(Motor1 <-99) Motor1 =-99;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (-Motor1));
	}
	else 
	{
		if(Motor1 >99) Motor1 = 99;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Motor1);
	}

	if(Motor2<0)
	{
		if(Motor2 <-99) Motor2=-99;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (-Motor2));
	}
	else
	{
		if(Motor2 >99) Motor2 =99;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,Motor2);
	}


}


