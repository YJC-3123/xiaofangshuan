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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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

uint16_t USART1_RX_STA=0;       //串口1接收状态标记：bit15，接收完成标志；bit14，	接收到0x0d；bit13~0，	接收到的有效字节数目
uint8_t USART1_RX_BUF[USART_REC_LEN];     //串口1完整数据接收缓冲
uint8_t USART1_TX_BUF[USART_REC_LEN];     //串口1完整数据发送缓冲
uint8_t u1_aRxBuffer[RXBUFFERSIZE];		//HAL库使用的串口1单字节接收缓冲

uint16_t USART2_RX_STA=0;       //串口2接收状态标记：bit15，	接收完成标志；bit14，	接收到0x0d；bit13~0，	接收到的有效字节数目
uint8_t USART2_RX_BUF[USART_REC_LEN];     //串口2完整数据接收缓冲
uint8_t USART2_TX_BUF[USART_REC_LEN];     //串口2完整数据发送缓冲
uint8_t u2_aRxBuffer[RXBUFFERSIZE];		//HAL库使用的串口2单字节接收缓冲

uint16_t LPUSART1_RX_STA=0;       //串口LP1接收状态标记：bit15，	接收完成标志；bit14，	接收到0x0d；bit13~0，	接收到的有效字节数目
uint8_t LPUSART1_RX_BUF[USART_REC_LEN];     //串口LP1完整数据接收缓冲
uint8_t LPUSART1_TX_BUF[USART_REC_LEN];     //串口LP1完整数据发送缓冲
uint8_t lp1_aRxBuffer[RXBUFFERSIZE];		//HAL库使用的串口LP1单字节接收缓冲
						 
extern lis3dh_t g_lis3dh;
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
	lis3dh_init(&g_lis3dh, &hi2c1, lis3dh_buffer, 6);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	uint8_t len;
	uint16_t times=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//收到定位模块串口消息则通过串口1发送到上位机
//		if(LPUSART1_RX_STA&0x8000)
//		{
//			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\ngnss msg is: \r\n",strlen("\r\ngnss msg is: \r\n"),1000);	//发送接收到的数据
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束
//			HAL_Delay(5);
//			len=LPUSART1_RX_STA&0x3fff;//得到此次接收到的数据长度
//			HAL_UART_Transmit(&huart1,(uint8_t*)LPUSART1_RX_BUF,len,1000);	//发送接收到的数据
//			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束
//			LPUSART1_RX_STA=0;
//		}
//		else
//		{
//			times++;
//			if(times == 65535)
//				times = 0;
//			HAL_Delay(100);
//		}
		//上位机发送给串口1的信息直接发给定位模块串口
		if(USART1_RX_STA&0x8000){
			HAL_UART_Transmit(&huart1,(uint8_t*)"\r\npc-mcu rev is: \r\n",strlen("\r\npc-mcu rev is: \r\n"),1000);	//发送接收到的数据
			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束
			len=USART1_RX_STA&0x3fff;//得到此次接收到的数据长度
			//PC消息回显
			HAL_Delay(5);
			HAL_UART_Transmit(&huart1,(uint8_t*)USART1_RX_BUF,len,1000);
			while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束
			//PC消息发给定位模块
//			HAL_Delay(5);
//			HAL_UART_Transmit(&hlpuart1,(uint8_t*)USART1_RX_BUF,len,1000);
//			while(__HAL_UART_GET_FLAG(&hlpuart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束			
			
			USART1_RX_STA=0;
		}else
		{
			times++;
			if(times == 65535)
				times = 0;
			HAL_Delay(10);   
		}

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
