/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
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

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern GNRMC GPS;
extern lis3dh_t g_lis3dh;

extern uint8_t NB_4G_State;
extern uint8_t Water_State;
extern float Voleage;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
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
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t timeout=0;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
		
	timeout=0;
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//等待就绪
	{
		timeout++;////超时处理
    if(timeout>HAL_MAX_DELAY) break;		
	}
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1, (uint8_t *)u1_aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
	
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles LPUART1 global interrupt / LPUART1 wake-up interrupt through EXTI line 28.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */
	uint32_t timeout=0;
  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */
	timeout=0;
  while (HAL_UART_GetState(&hlpuart1) != HAL_UART_STATE_READY)//等待就绪
	{
		timeout++;////超时处理
    if(timeout>HAL_MAX_DELAY) break;		
	}
	timeout=0;
	while(HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)lp1_aRxBuffer, RXBUFFERSIZE) != HAL_OK)//一次处理完成之后，重新开启中断并设置RxXferCount为1
	{
	 timeout++; //超时处理
	 if(timeout>HAL_MAX_DELAY) break;	
	}
		
  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//霍尔开关被触发
	if(GPIO_Pin == GPIO_PIN_0)
	{
		HAL_Delay(100);	//消抖
		//蜂鸣器
		BEEP_On(1000);
		
		//获取模式选择状态
		NB_4G_State = Get_Mode_State();
		
		//获取当前位置信息，时间
//		GNSS_data_parse(uint8_t * buff_t)
		
		//获取姿态信息
		lis3dh_get_xyz(&g_lis3dh);
		
		//获取电池电量信息
		Voleage = get_voleage();
		
		//获取水浸状态
		Water_State = Get_Water_State();
		
		//向服TCP服务器发送信息
		uint8_t msg[256];
		sprintf(msg, "NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:,Lat:,Water_P:",NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z);
		send_msg_tcp_server(msg);
	}
	else if(GPIO_Pin == GPIO_PIN_5)		//蓝牙模块有数据需要传输
	{
		
	}
}


/**
  * @brief 串口中断外部回调
  * @note  中断初始化时设置的接收字节数为1，当收到1字节数据触发中断，并将缓冲区的内容搬移到完整数据缓冲区
  * @param huart   UART句柄.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//如果是串口1
	{
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(u1_aRxBuffer[0]!=0x0a)
					USART1_RX_STA=0;//接收错误,重新开始
				else 
					USART1_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D,则将HAL接收函数的单字节缓冲内容搬到完整数据缓冲
			{	
				if(u1_aRxBuffer[0]==0x0d)
				{
					USART1_RX_STA|=0x4000;
				}
				else
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=u1_aRxBuffer[0] ;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))
							USART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}

	}
	else if(huart->Instance == LPUART1) {
		if((LPUSART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(LPUSART1_RX_STA&0x4000)//接收到了0x0d
			{
				if(lp1_aRxBuffer[0]!=0x0a)
					LPUSART1_RX_STA=0;//接收错误,重新开始
				else 
					LPUSART1_RX_STA|=0x8000;	//接收完成了
		
			}
			else //还没收到0X0D,则将HAL接收函数的单字节缓冲内容搬到完整数据缓冲
			{	
				if(lp1_aRxBuffer[0]==0x0d)
				{
					LPUSART1_RX_STA|=0x4000;
				}
				else
				{
					LPUSART1_RX_BUF[LPUSART1_RX_STA&0X3FFF]=lp1_aRxBuffer[0] ;
					LPUSART1_RX_STA++;
					if(LPUSART1_RX_STA>(USART_REC_LEN-1))
							LPUSART1_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}
	}
}

/* USER CODE END 1 */
