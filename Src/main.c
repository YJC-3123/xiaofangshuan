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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
										 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t USART1_RX_STA=0;       //����1����״̬��ǣ�bit15��������ɱ�־��bit14��	���յ�0x0d��bit13~0��	���յ�����Ч�ֽ���Ŀ
uint8_t USART1_RX_BUF[USART_REC_LEN];     //����1�������ݽ��ջ���
uint8_t USART1_TX_BUF[USART_REC_LEN];     //����1�������ݷ��ͻ���
uint8_t u1_aRxBuffer[RXBUFFERSIZE];		//HAL��ʹ�õĴ���1���ֽڽ��ջ���

uint16_t USART2_RX_STA=0;       //����2����״̬��ǣ�bit15��	������ɱ�־��bit14��	���յ�0x0d��bit13~0��	���յ�����Ч�ֽ���Ŀ
uint8_t USART2_RX_BUF[USART_REC_LEN];     //����2�������ݽ��ջ���
uint8_t USART2_TX_BUF[USART_REC_LEN];     //����2�������ݷ��ͻ���
uint8_t u2_aRxBuffer[RXBUFFERSIZE];		//HAL��ʹ�õĴ���2���ֽڽ��ջ���

uint16_t LPUSART1_RX_STA=0;       //����LP1����״̬��ǣ�bit15��	������ɱ�־��bit14��	���յ�0x0d��bit13~0��	���յ�����Ч�ֽ���Ŀ
uint8_t LPUSART1_RX_BUF[USART_REC_LEN];     //����LP1�������ݽ��ջ���
uint8_t LPUSART1_TX_BUF[USART_REC_LEN];     //����LP1�������ݷ��ͻ���
uint8_t lp1_aRxBuffer[RXBUFFERSIZE];		//HAL��ʹ�õĴ���LP1���ֽڽ��ջ���

uint8_t REC_FLAG = 0;		//���������Ƿ񱻰��±�־
uint8_t NB_4G_State;		//ͨѶģʽ״̬��¼:1Ϊ4G��0ΪNB
uint8_t Water_State;		//ˮ��״̬��¼��1Ϊˮ����0Ϊδˮ��
float Voleage;	//��ص���
uint8_t Water_Pre[10];		//ˮѹ
bool isGetWP;		//�Ƿ���Զ������ˮѹ���ݱ�־
uint8_t CPIN_FLAG;	//4Gģ���ʼ�����sim����־
uint8_t CREG_FLAG;	//4Gģ���ʼ�����CSҵ���־
uint8_t CGREG_FLAG;	//4Gģ���ʼ�����PSҵ���־

extern lis3dh_t g_lis3dh;	//��̬��Ϣ�ṹ��
uint8_t it0_count = 0;	//�ⲿ�����жϼ��������ڷ�ֹ�������ش�������

//��ʼ��̬
int32_t g_init_x;
int32_t g_init_y;
int32_t g_init_z;



extern UART_HandleTypeDef huart1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_StatusTypeDef status = lis3dh_init(&g_lis3dh, &hi2c1, lis3dh_buffer, 6);
	if(status != HAL_OK)
		print_u1("lis3dh_init fail\r\n");
	else
		print_u1("lis3dh_init ok\r\n");
	
	nb_module_init();		//����ͨ��ģ���ʼ��
	ble_mode_init(4);	//������������Ϊ����һ��ģʽ
	BEEP_On(1000);
	HAL_Delay(1000);
	print_u1("runing...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//���ԣ��յ���λģ�鴮����Ϣ��ͨ������1���͵���λ��
//		if(LPUSART1_RX_STA&0x8000)
//		{
//			print_u1("\r\ngnss msg is: \r\n");
//			uint16_t len=LPUSART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			HAL_UART_Transmit(&huart1,(uint8_t*)LPUSART1_RX_BUF,len,1000);	//���ͽ��յ�������
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//�ȴ����ͽ���
//			LPUSART1_RX_STA=0;
//		}

		//���ԣ���λ�����͸�����1����Ϣֱ�ӷ�����λģ�鴮��
//		if(USART1_RX_STA&0x8000){
//			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\npc-mcu rev is: \r\n",strlen("\r\npc-mcu rev is: \r\n"),1000);	//���ͽ��յ�������
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//�ȴ����ͽ���
//			len=USART1_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			//PC��Ϣ����
//			HAL_Delay(5);
//			HAL_UART_Transmit(&huart1,(uint8_t*)USART1_RX_BUF,len,1000);
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//�ȴ����ͽ���			
//			USART1_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times == 65535)
//				times = 0;
//			HAL_Delay(10);   
//		}

		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_IncTick(void)
{
	if(uwTickFreq==0)
	{
		uwTickFreq = HAL_TICK_FREQ_DEFAULT;
	}
  uwTick += uwTickFreq;
}


/*
����0��1��2��3-->�û����ӡ��Ͽ���"GET"��"START"��"STOP"����
����4��5��δ��ˮѹ���������Ϣ�л�ȡ��ˮѹ���ݡ��ɹ�����ˮѹ����
*/
uint8_t process_remote_ble_recv(uint8_t *buffer)
{
		//�ж��Ƿ����û�������������
		char *check_connect = strstr((char*)buffer, "TTM:CONNECTED");
		if(check_connect != NULL)
			return 0;
		char *check_disconnect = strstr((char*)buffer, "TTM:DISCONNECT");
		if(check_disconnect != NULL)
			return 1;
		char *check_get = strstr((char*)buffer, "CHECK");
		if(check_get != NULL)
			return 2;
		char *check_start = strstr((char*)buffer, "START");
		if(check_start != NULL)
			return 3;
		char *check_stop = strstr((char*)buffer, "STOP");
		if(check_stop != NULL)
			return 4;
		//�ж��Ƿ���ˮѹС���������ص�ˮѹ���ݣ���ʽΪ"#x.xx#",��Ϊˮѹ���������ö�Ӧȫ�ֱ�־��ȫ������
		char* pidx0, *pidx1;
		pidx0 = strchr((char *)buffer, '#');
		if(pidx0 == NULL)
			return 5;
		pidx1 = strrchr((char*)buffer,'#');
		if(pidx1 == NULL)
			return 5;
		isGetWP = true;
		uint8_t len = pidx1 - pidx0 -1;
		strncpy((char*)Water_Pre, pidx0+1, len);
		return 6;
}

bool check_error(void){
	//��̬ƫ��
//	if((double)abs(g_lis3dh.x - g_init_x)/(double)g_init_x >= 0.3)
//		return true;
	if(((double)abs(g_lis3dh.y - g_init_y)/(double)g_init_y) >= 0.2)
		return true;
	if(((double)abs(g_lis3dh.z - g_init_z)/(double)g_init_z) >= 0.2)
		return true;
	//ˮ��
	if(Water_State == 1)
		return true;
	//��ѹ����
	if(Voleage < 3.0)
		return true;
	//ˮѹ����
	char* WP_endptr;
	double WP_val = strtod((const char*)Water_Pre, &WP_endptr);
	if(WP_endptr != (char*)Water_Pre)
	{
		if(WP_val <= 0.06 || WP_val >= 0.14)
			return true;
	}
	return false;
}

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
