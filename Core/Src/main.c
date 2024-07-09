/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/**/
#include "board.h"
/**/

extern uint8_t Usart1_ReadBuf[256]; //����1 ��������
extern uint8_t Usart1_ReadCount; //����1 �����ֽڼ���

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*RTT����*/
rt_thread_t all_thread = RT_NULL;

static void all_thread_entry(void *p)
{
    all_control();
}
/*RTT����*/

uint8_t OledString[35];


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int c, FILE *f)
{
		uint8_t ch[1] = {c};
		HAL_UART_Transmit(&huart1,ch,1,0x100);
		
		return c;
}


//�жϷ������һ֡����
uint8_t Usart_WaitReasFinish(void)
{
 static uint16_t Usart_LastReadCount = 0;//��¼�ϴεļ���ֵ
 if(Usart1_ReadCount == 0)
 {
		Usart_LastReadCount = 0;
		return 1;//��ʾû���ڽ�������
 }
 if(Usart1_ReadCount == Usart_LastReadCount)//�����μ���ֵ�����ϴμ���ֵ
 {
		Usart1_ReadCount = 0;
		Usart_LastReadCount = 0;
		return 0;//�Ѿ����������
 }
 Usart_LastReadCount = Usart1_ReadCount;
 return 2;//��ʾ���ڽ�����
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    all_thread = rt_thread_create("all_thread",
                                 all_thread_entry, RT_NULL,
                                 512,
                                 5,
                                 20);
    if(all_thread != RT_NULL)
        rt_thread_startup(all_thread);
    else
        return -1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
