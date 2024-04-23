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
extern uint8_t REC_FLAG;
extern GNRMC GPS;
extern lis3dh_t g_lis3dh;

bool g_ble_recvbusy;

extern uint8_t NB_4G_State;
extern uint8_t Water_State;
extern float Voleage;
extern bool isGetWP;
extern uint8_t Water_Pre[10];

extern uint8_t it0_count;


uint8_t default_lon[] = "113.20E";
uint8_t default_lat[] = "23.9N";
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

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	uint32_t timeout=0;
  while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)//�ȴ�����
	{
		timeout++;////��ʱ����
    if(timeout>HAL_MAX_DELAY) break;		
	}
	timeout=0;
	while(HAL_UART_Receive_IT(&huart1, (uint8_t *)u1_aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
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

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */
	uint32_t timeout=0;
  while (HAL_UART_GetState(&hlpuart1) != HAL_UART_STATE_READY)//�ȴ�����
	{
		timeout++;////��ʱ����
    if(timeout>HAL_MAX_DELAY) break;		
	}
	timeout=0;
	while(HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)lp1_aRxBuffer, RXBUFFERSIZE) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
		
  /* USER CODE END LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)		//�������ر�����
	{
		//�˴���ֹ��������
		if(it0_count % 2 != 0){
			it0_count++;
			return;
		}
		if(it0_count == 100 || it0_count == 101)
			it0_count = 0;
		++it0_count;
		HAL_Delay(10);	//����
		//������
		BEEP_On(1000);
		//��ȡģʽѡ��״̬
		NB_4G_State = Get_Mode_State();	
		//��ȡ��ǰλ����Ϣ��ʱ��
		//GNSS_data_parse(uint8_t * buff_t);
		//��ȡ��̬��Ϣ
		lis3dh_get_xyz(&g_lis3dh);
		//��ȡ��ص�����Ϣ
		Voleage = get_voleage();
		//��ȡˮ��״̬
		Water_State = Get_Water_State();
		//����Զ�����������ȡˮѹ
		isGetWP = false;
		connet_remote_ble();
		HAL_Delay(1000);		//Todo���˴�ͨ���ӳٱ�֤���ӳɹ���Ӧ�Ż�Ϊ�������״̬
		send_remote_ble("#GET_REQ#");
		//�ȴ��յ�ˮѹ��Ϣ���������ݽ����ж����ȼ�����ڱ��ж����ȼ�
		for(uint8_t i=0;i<30;i++)
		{
			HAL_Delay(100);
			if(isGetWP == true)
				break;
		}
		uint8_t msg[256];
		sprintf((char*)msg, "#NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,Water_P:%s#",
			NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre);
		send_msg_nbtcp_server(msg);
		//���������㲥�ȴ����� //Todo:�˴�ͨ������Ӳ������bleģ��en�ſ����㲥��������������ظ���Ϣ�ᵼ���ж�Ƕ��
		send_msg_ble("TTM:ADV-1");
	}
	else if(GPIO_Pin == GPIO_PIN_5)		//����ģ����Ҫ��������
	{
		g_ble_recvbusy = true;
		uint8_t ble_recv_buffer[20];
		memset(ble_recv_buffer,0,sizeof(ble_recv_buffer));
		HAL_UART_Receive(&huart2, ble_recv_buffer, 20, 1000);
		__HAL_UART_FLUSH_DRREGISTER(&huart1);
		//����Դ��ڷ��ͽ��յ�����Ϣ
		uint8_t debug_msg[128];
		sprintf((char*)debug_msg,"BLE:%s\n",ble_recv_buffer);
		print_u1(debug_msg);
		//��������
		uint8_t msg_status = process_remote_ble_recv(ble_recv_buffer);
		if(msg_status == 0)		//�û�����
		{
			return;
		}
		else if(msg_status == 1)
		{

			send_msg_ble("TTM:ADV-0");	//�û��Ͽ����ӹرչ㲥
		}
		else if(msg_status == 2)	//�յ�GET����
		{	
			NB_4G_State = Get_Mode_State();		//��ȡģʽѡ��״̬
			//GNSS_data_parse(uint8_t * buff_t);	//��ȡ��ǰλ����Ϣ��ʱ��
			lis3dh_get_xyz(&g_lis3dh);		//��ȡ��̬��Ϣ
			Voleage = get_voleage();		//��ȡ��ص�����Ϣ
			Water_State = Get_Water_State();	//��ȡˮ��״̬
			uint8_t msg[200];
			sprintf((char*)msg, "[INFO]NB_4G:%d,Water:%d,Vol:%f,X:%d,Y:%d,Z:%d,Lon:%s,Lat:%s,WP:%s",
				NB_4G_State,Water_State,Voleage, g_lis3dh.x,g_lis3dh.y,g_lis3dh.z,default_lon, default_lat,Water_Pre);
			send_user_ble(msg);
		}else if(msg_status == 3)
		{	//�յ�START����

		}else if(msg_status == 4)
		{	//�յ�STOP����

		}
		g_ble_recvbusy = false;
	}
}

/**
  * @brief �����ж��ⲿ�ص�
  * @note  �жϳ�ʼ��ʱ���õĽ����ֽ���Ϊ1�����յ�1�ֽ����ݴ����жϣ����������������ݰ��Ƶ��������ݻ�����
  * @param huart   UART���.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
		if((USART1_RX_STA&0x8000)==0)//����δ���
		{
			if(USART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(u1_aRxBuffer[0]!=0x0a)
					USART1_RX_STA=0;//���մ���,���¿�ʼ
				else 
					USART1_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D,��HAL���պ����ĵ��ֽڻ������ݰᵽ�������ݻ���
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
							USART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}
	}
	else if(huart->Instance == LPUART1) {
		if((LPUSART1_RX_STA&0x8000)==0)//����δ���
		{
			if(LPUSART1_RX_STA&0x4000)//���յ���0x0d
			{
				if(lp1_aRxBuffer[0]!=0x0a)
					LPUSART1_RX_STA=0;//���մ���,���¿�ʼ
				else 
					LPUSART1_RX_STA|=0x8000;	//���������
		
			}
			else //��û�յ�0X0D,��HAL���պ����ĵ��ֽڻ������ݰᵽ�������ݻ���
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
							LPUSART1_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}
	} 
}

/* USER CODE END 1 */
