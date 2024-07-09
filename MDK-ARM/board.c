/* ������Ӳ�����ͷ�ļ� */
#include "board.h"

extern uint8_t  OpenMV_Data[20];
float Motor1Pwm;
float Motor2Pwm;
short EncoderCNT1 = 0;
short EncoderCNT2 = 0;
float EncoderSpeed1 = 0;
float EncoderSpeed2 = 0;
extern PIDstructure PIDMotorSpeed1;
extern PIDstructure PIDMotorSpeed2;
extern uint16_t pingpang_count;
float  battery_voltage[40];

extern uint16_t x_error ;
extern uint16_t width   ;
extern uint16_t hight   ;
extern uint16_t y_error ;
extern uint16_t pixel   ;
extern int16_t goal_speed;
extern int16_t turn_error;
extern int16_t speed_error;
extern uint8_t find_flag;

extern uint8_t Usart1_ReadBuf[256]; //����1 ��������

float p,i,d,a,b;//pid������a��bΪĿ��ֵ

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 1024
/* ���ڲ�SRAM�������һ���־�̬�ڴ�����Ϊrtt�Ķѿռ䣬��������Ϊ4KB */
static uint32_t rt_heap[RT_HEAP_SIZE];
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}
 
RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif
 
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



/**
  * @brief  ������Ӳ����ʼ������
  * @param  ��
  * @retval ��
  *
  * @attention
  * RTT�ѿ�������صĳ�ʼ������ͳһ�ŵ�board.c�ļ���ʵ�֣�
  * ��Ȼ���������Щ����ͳһ�ŵ�main.c�ļ�Ҳ�ǿ��Եġ�
  */
void rt_hw_board_init()
{
    /* ��ʼ��SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );	
    
	/* Ӳ��BSP��ʼ��ͳͳ�����������LED�����ڣ�LCD�� */
      /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	//50ms��ʱ�ж�
	HAL_TIM_Base_Start_IT(&htim5);
	//��������1�����ж�
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE); 
	//OpenMV����
	HAL_UART_Receive_DMA(&huart3,OpenMV_Data,20);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	//�������ɼ�
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	//�������
	HAL_TIM_Base_Start(&htim3); 
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	//���
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	//json�����ṹ��
	
	//ADC�ɼ�
	adc_dma_start();
	PID_Init();
	OLED_Init();
	OLED_Clear();
	
	
	
	//__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
/* ���������ʼ������ (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
    
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
	rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
    
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

void all_control(void)
{
    while(1)
    {
            cJSON *cJsonData ,*cJsonVlaue;
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1700);
        oled_show();

//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 500);
//		HAL_Delay(500);
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1200);
//			HAL_Delay(500);
		
		

		if(find_flag == 0)
		{

				PIDMotorSpeed1.Target_value   = 0.5;
				PIDMotorSpeed2.Target_value   = -0.5;
				Motor_Set(PID_realize(&PIDMotorSpeed1,EncoderSpeed1),PID_realize(&PIDMotorSpeed2,EncoderSpeed2));
			
		}
			
		
		if(Usart_WaitReasFinish() == 0)//�Ƿ�������
        {

		 cJsonData = cJSON_Parse((const char *)Usart1_ReadBuf);
		
		 if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
		 {
			 
				cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p"); 
				p = cJsonVlaue->valuedouble;
				PIDMotorSpeed2.Kp = p;
		 
		 }
		
		 if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
		 {
		
				cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i"); 
				i = cJsonVlaue->valuedouble;
				PIDMotorSpeed2.Ki = i;
		 
		 }
		 
		 if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
		 {
		
				cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d"); 
				d = cJsonVlaue->valuedouble;
				PIDMotorSpeed2.Kd = d;
		 
		 }
		 
		 if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
		 {
			 
				cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a"); 
				a = cJsonVlaue->valuedouble;
				PIDMotorSpeed1.Target_value =a;
		 
		 }
		 
		 		 if(cJSON_GetObjectItem(cJsonData,"b") !=NULL)
		 {
			 
				cJsonVlaue = cJSON_GetObjectItem(cJsonData,"b"); 
				b = cJsonVlaue->valuedouble;
				PIDMotorSpeed2.Target_value =b;
		 
		 }
		 
		 if(cJsonData != NULL){
		 cJSON_Delete(cJsonData);//�ͷſռ䡢���ǲ���ɾ��cJsonVlaue��Ȼ�� �����쳣����
		 }
		 memset(Usart1_ReadBuf,0,255);//��ս���buf��ע�����ﲻ��ʹ��strlen 
	}
	
	printf("P:%.3f I:%.3f D:%.3f A:%.3f\r\n",p,i,d,a);
	

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }
}
 
/**
  * @brief  SysTick�жϷ�����
  * @param  ��
  * @retval ��
  *
  * @attention
  * SysTick�жϷ������ڹ̼����ļ�stm32f10x_it.c��Ҳ�����ˣ�������
  * ��board.c���ֶ���һ�Σ���ô�����ʱ�������ظ�����Ĵ��󣬽��
  * �����ǿ��԰�stm32f10x_it.c�е�ע�ͻ���ɾ�����ɡ�
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
    
    /* �����ж� */
    rt_interrupt_enter();
 
    /* ����ʱ�� */
    rt_tick_increase();
 
    /* �뿪�ж� */
    rt_interrupt_leave();
}
