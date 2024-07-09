/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "PID.H"
#include "niming.h"
#include "cJSON.h"
#include <string.h>
#include "motor.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Usart1_ReadBuf[256]; //串口1 缓冲数组
uint8_t Usart1_ReadCount = 0; //串口1 接收字节计数
uint16_t pingpang_count = 0;
uint16_t Count1 = 0;
float voltage_difference = 0;
float electric_quantity = 0.0;

float goal_speed = 0;
int16_t turn_error = 0;
float speed_error = 0;
uint16_t pixel   = 0;

uint8_t  OpenMV_Data[20] = {0};
unsigned char DMA_rx2_len;
uint16_t x_error = 0;
uint16_t width   = 0;
uint16_t hight   = 0;
uint16_t y_error = 0;
uint8_t find_flag= 0;
float goal_speed1 = 0;
float goal_speed2 = 0;

//extern int EncoderSpeed1;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern short EncoderCNT1;
extern short EncoderCNT2;
extern TIM_HandleTypeDef htim1;
extern PIDstructure PIDMotorSpeed1;
extern PIDstructure PIDMotorSpeed2;
extern PIDstructure OpenMV_Pixel;
extern PIDstructure Turnspeed;
extern float EncoderSpeed1;
extern float EncoderSpeed2;
extern float  battery_voltage[40];
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */


/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */


/**
  * @brief This function handles System tick timer.
  */

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

	pingpang_count++;
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
		 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE))//判断huart1 是否读到字节
		 {
		 if(Usart1_ReadCount >= 255) Usart1_ReadCount = 0;
		 HAL_UART_Receive(&huart1,&Usart1_ReadBuf[Usart1_ReadCount++],1,1000);
		 }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	DMA_rx2_len = 0;
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		HAL_UART_DMAStop(&huart3);
		DMA_rx2_len=20-__HAL_DMA_GET_COUNTER(huart3.hdmarx);
		HAL_UART_Transmit_DMA(&huart3,OpenMV_Data,DMA_rx2_len);
		if(OpenMV_Data[0] == 0x2C)
		{
			
			if(OpenMV_Data[1] == 0x12)
			{
					x_error =OpenMV_Data[2];  
					width   =OpenMV_Data[3];  
					hight   =OpenMV_Data[4];  
					y_error =OpenMV_Data[5];
					find_flag	= OpenMV_Data[6];

			}
		
		}
		if(OpenMV_Data[0] == 0x2C)
		{
			
			if(OpenMV_Data[1] == 0x11)
			{
				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
					find_flag	= OpenMV_Data[2];
			}
		
		}

		
		HAL_UART_Receive_DMA(&huart3,OpenMV_Data,20);
	
	}
	
	
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
		EncoderCNT1 = -(short)__HAL_TIM_GetCounter(&htim1);
		EncoderCNT2 = -(short)__HAL_TIM_GetCounter(&htim2);	
		__HAL_TIM_SetCounter(&htim1,0);
		__HAL_TIM_SetCounter(&htim2,0);		
		EncoderSpeed1 = (float)EncoderCNT1*20/1320;
		EncoderSpeed2 = (float)EncoderCNT2*20/11/30/4;//乘以20是1s的，除以11是每圈11脉冲，30是减速比，4是计数倍频。整体计算的是每秒多少圈
//		Motor_Set(PID_realize(&PIDMotorSpeed1,EncoderSpeed1),PID_realize(&PIDMotorSpeed2,EncoderSpeed2));
	
		pixel = width*hight;
		goal_speed = PID_realize(&OpenMV_Pixel,pixel);
		turn_error = x_error-80;
		speed_error = PID_realize(&Turnspeed,turn_error);
	
		if(goal_speed>100) goal_speed     =  100;
		if(goal_speed<-100) goal_speed    = -100;
		if(speed_error>100) speed_error   =  100;
		if(speed_error<-100) speed_error  = -100;
	
		goal_speed1 = goal_speed-speed_error;
		goal_speed2 = goal_speed+speed_error;
	
		if(goal_speed1>30) goal_speed1   = 30;
		if(goal_speed1<-30) goal_speed1  = -30;
		if(goal_speed2>30) goal_speed2   = 30;
		if(goal_speed2<-30) goal_speed2  = -30;	

			if(find_flag)
			{

					Motor_Set(goal_speed1,goal_speed2);
					
			}


		
		Count1++;
		if(Count1 == 1200)
		{
			Count1 = 0;
			data_pros(battery_voltage);
			voltage_difference = battery_voltage[0] - 11;
			electric_quantity = voltage_difference / 1.7;
			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_2);
		}
		
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
