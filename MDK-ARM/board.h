#ifndef __BOARD_H
#define __BOARD_H

/* RT-Thread相关头文件 */
#include <rthw.h>
#include <rtthread.h>

#include "stm32f1xx_hal.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "PID.H"
#include "niming.h"
#include "cJSON.h"
#include <string.h>
#include "motor.h"
#include "oled.h"
/* USER CODE END Includes */

void rt_hw_board_init();
void SysTick_Handler(void);

void all_control(void);

#endif
