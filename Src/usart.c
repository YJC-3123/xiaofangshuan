/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#define DELAY_TIME 1000
GNRMC GPS;
//TCP目标服务器的IP需要为公网IP，测试使用http://tt.ai-thinker.com:8000/ttcloud 给出的IP
uint8_t tcp_server_ip[] = "cpolard.26.tcp.cpolar.top";
uint16_t tcp_server_port = 11728;
uint8_t tcp_client_socket = 0;

uint8_t REMOTE_MAC[64] = "0xBA03454151C1";

extern uint8_t CPIN_FLAG;
extern uint8_t CREG_FLAG;
extern uint8_t CGREG_FLAG;

/* USER CODE END 0 */

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
	HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)lp1_aRxBuffer, RXBUFFERSIZE);//开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  /* USER CODE END LPUART1_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t *)u1_aRxBuffer, RXBUFFERSIZE);//开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspInit 0 */

  /* USER CODE END LPUART1_MspInit 0 */
    /* LPUART1 clock enable */
    __HAL_RCC_LPUART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**LPUART1 GPIO Configuration
    PB10     ------> LPUART1_TX
    PB11     ------> LPUART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LPUART1 interrupt Init */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspInit 1 */

  /* USER CODE END LPUART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==LPUART1)
  {
  /* USER CODE BEGIN LPUART1_MspDeInit 0 */

  /* USER CODE END LPUART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
    PB10     ------> LPUART1_TX
    PB11     ------> LPUART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* LPUART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspDeInit 1 */

  /* USER CODE END LPUART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void print_u1(uint8_t *msg){
	if(msg != NULL){
		HAL_UART_Transmit(&huart1,(uint8_t*)msg,strlen((char*)msg),1000);
		while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)!=SET);		HAL_Delay(5);//等待发送结束
	}
}

//解析GNSS数据
void GNSS_data_parse(uint8_t * buff_t) {
	uint16_t add = 0, x = 0, y = 0, z = 0, i = 0;
	uint32_t Time = 0, times = 1.0;
	long double latitude = 0, longitude = 0;
	GPS.Status = 0;
  GPS.Time_H = 0;
  GPS.Time_M = 0;
  GPS.Time_S = 0;

	buff_t[USART_REC_LEN - 1] = '\0';
	add = 0;
	while (add < USART_REC_LEN)
	{
		if (buff_t[add] == '$' && buff_t[add + 1] == 'G' && (buff_t[add + 2] == 'N' || buff_t[add + 2] == 'P') && buff_t[add + 3] == 'R' && buff_t[add + 4] == 'M' && buff_t[add + 5] == 'C')
		{
			x = 0;
			for (z = 0; x < 12; z++)	// x=12 共读取12个key
			{
				if (buff_t[add + z] == '\0')
						break;
				if (buff_t[add + z] == ',')
				{
					x++;
					if (x == 1) // UTC时间
					{
						Time = 0;
						for (i = 0; buff_t[add + z + i + 1] != '.'; i++)
						{
								if (buff_t[add + z + i + 1] == '\0')
								{
										break;
								}
								if (buff_t[add + z + i + 1] == ',')
										break;
								Time = (buff_t[add + z + i + 1] - '0') + Time * 10;
						}

						GPS.Time_H = Time/10000 + 8;
						GPS.Time_M = Time/100%100;
						GPS.Time_S = Time%100;
						if (GPS.Time_H >= 24)
								GPS.Time_H = GPS.Time_H - 24;
					}
					else if (x == 2) // 定位系统状态判断
					{
						if (buff_t[add + z + 1] == 'A')
						{
								GPS.Status = 1;
						}
						else
						{
								GPS.Status = 0;
						}
					}
					else if (x == 3) // 维度
					{
							latitude = 0;
							// If you need to modify, please re-edit the calculation method below.
							for (i = 0; buff_t[add + z + i + 1] != ','; i++)
							{
									if (buff_t[add + z + i + 1] == '\0')
									{
											break;
									}
									if (buff_t[add + z + i + 1] == '.')
									{
											y = i;
											continue;
									}
									latitude = (buff_t[add + z + i + 1] - '0') + latitude * 10;
							}
							times = 1.0;
							while (i >= y)
							{
									times = times * 10.0;
									i--;
							}
							GPS.Lat = (double)latitude / (double)times;
							// GPS.Lat = latitude/1000000.0;
					}
					else if (x == 4) // 维度方向
					{
							GPS.Lat_area = buff_t[add + z + 1];
					}
					else if (x == 5) // 经度
					{
							longitude = 0;
							for (i = 0; buff_t[add + z + i + 1] != ','; i++)
							{
									if (buff_t[add + z + i + 1] == '\0')
									{
											break;
									}
									if (buff_t[add + z + i + 1] == '.')
									{
											y = i;
											continue;
									}

									longitude = (buff_t[add + z + i + 1] - '0') + longitude * 10;
							}
							times = 1.0;
							while (i >= y)
							{
									times = times * 10.0;
									i--;
							}
							GPS.Lon = (double)longitude / (double)times;
							// GPS.Lon = longitude/1000000.0;
					}
					else if (x == 6) // 经度方向
					{
							GPS.Lon_area = buff_t[add + z + 1];
					}
				}
			}
			add = 0;
			break;
		}
		if (buff_t[add + 5] == '\0')
		{
			add = 0;
			break;
		}
		add++;
		if (add > USART_REC_LEN)
		{
			add = 0;
			break;
		}
	}
}



void nb_module_init(){
	//重置模块
	set_reset_net_module(0);
	HAL_Delay(500);
	set_reset_net_module(1);
	HAL_Delay(500);
	//模块退出睡眠模式
	set_reset_net_psm(0);
	HAL_Delay(500);

	//关闭回显
	print_u1("ATE0\r\n");
	HAL_Delay(DELAY_TIME);
	//禁止休眠(此处可能之前已经进入睡眠，故发两次，需优化)
	print_u1("AT+QSCLK=0\r\n");
	HAL_Delay(DELAY_TIME);
	print_u1("AT+QSCLK=0\r\n");
	HAL_Delay(DELAY_TIME);
	//配置数据格式为文本
	print_u1("AT+QICFG=\"dataformat\",0,0\r\n");
	HAL_Delay(DELAY_TIME);
}

HAL_StatusTypeDef connect_tcp_server(void){
	//将所有可能存在的连接关闭
	uint8_t msg0[30];
	for(uint8_t socket=0;socket<1;++socket){
		sprintf((char *)msg0,"AT+QICLOSE=%d\r\n",socket);
		print_u1(msg0);
		HAL_Delay(DELAY_TIME);
	}
	//连接TCP服务器，使用socket=0
	uint8_t msg1[128];
	sprintf((char *)msg1, "AT+QIOPEN=0,%d,\"TCP\",\"%s\",%d\r\n",tcp_client_socket,(char*)tcp_server_ip, tcp_server_port);
	print_u1(msg1);
	HAL_Delay(DELAY_TIME);
	//Todo：此处可通过发送AT+QISTATE=1,0并检查返回内容判断连接是否成功建立
	return HAL_OK;
}


//以文本形式向TCP服务器发送信息(msg信息中不需要带\r\n)
void send_msg_nbtcp_server(uint8_t* msg){
	//先建立TCP连接
	connect_tcp_server();
	HAL_Delay(DELAY_TIME);
	//发送数据
	uint16_t len = strlen((char*)msg);
	uint8_t send_msg0[300];
	sprintf((char*)send_msg0, "AT+QISEND=0,%d,\"%s\"\r\n", len, msg);
	print_u1(send_msg0);
	HAL_Delay(DELAY_TIME);
	//Todo：此处需要完善判断数据是否发送成功
	//断开连接
	uint8_t send_msg1[30];
	sprintf((char *)send_msg1, "AT+QICLOSE=%d\r\n",tcp_client_socket);
	print_u1(send_msg1);
	HAL_Delay(DELAY_TIME);
}

//4G模块初始化
void cat1_module_init(void){
	//关闭回显
	print_u1("ATE0\r\n");
	HAL_Delay(DELAY_TIME);
	//Todo：以下检查操作后续应添加检查成功判断
	//检查SIM卡
	print_u1("AT+CPIN?\r\n");
	HAL_Delay(DELAY_TIME);
	//查询CS业务状态
	print_u1("AT+CREG?\r\n");
	HAL_Delay(DELAY_TIME);
	//查询PS业务状态
	print_u1("AT+CGREG?\r\n");
	HAL_Delay(DELAY_TIME);
}

/*检查4G Cat1模块的返回信息，判断初始化过程中各指令是否正确执行*/
void check_cat1_recvmsg(uint8_t* recvmsg){
	if(recvmsg == NULL)
		return;
	if(strstr((char*)recvmsg, "CPIN: READY") != NULL)
		CPIN_FLAG = 1;
	if(strstr((char*)recvmsg, "CREG: 0,1") != NULL || strstr((char*)recvmsg, "CREG: 0,5") != NULL)
		CREG_FLAG = 1;
	if(strstr((char*)recvmsg, "CGREG: 0,1") != NULL || strstr((char*)recvmsg, "CGREG: 0,5") != NULL)
		CGREG_FLAG = 1;	
}

//连接4G TCP服务器并发送信息
void send_msg_cat1_tcp_server(uint8_t* msg){
	//Todo：以下检查操作后续应添加检查成功判断
	//配置场景1
	print_u1("AT+QICSGP=1,1,\"CMNET\","","",1\r\n");	//CMNET为中国移动APN
	HAL_Delay(DELAY_TIME);
	//激活场景1
	print_u1("AT+QIACT=1\r\n");
	HAL_Delay(DELAY_TIME);
	//透传模式连接TCP服务器
	uint8_t connect_msg[128];
	sprintf((char*)connect_msg, "AT+QIOPEN=1,%d,\"TCP\",\"%s\",%d,0,2\r\n", tcp_client_socket,(char*)tcp_server_ip, tcp_server_port);
	print_u1(connect_msg);
	HAL_Delay(DELAY_TIME);
	//发送消息
	print_u1(msg);
	HAL_Delay(DELAY_TIME);
	//断开连接
	print_u1("+++");
	HAL_Delay(DELAY_TIME);
	uint8_t disconnect_msg[20];
	sprintf((char*)disconnect_msg, "AT+QICLOSE=%d\r\n",tcp_client_socket);
	print_u1(disconnect_msg);
	HAL_Delay(DELAY_TIME);
	//场景去激活
	print_u1("AT+QIDEACT=1\r\n");
	HAL_Delay(DELAY_TIME);
}


/*从MCU向蓝牙模块串口发送信息*/
void send_msg_ble(uint8_t *msg){
		set_reset_brts(0);
		HAL_Delay(200);
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen((const char *)msg), 1000); // 发送数据
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != SET) HAL_Delay(5); // 等待发送结束
		__HAL_UART_FLUSH_DRREGISTER(&huart2);
	//		HAL_Delay(200);		//Todo:此处暂不考虑低功耗将brts拉高
	//	  set_reset_brts(1);
}

/*设置当前蓝牙模块工作模式：有效值0~5，默认为2（从机模式）*/
void ble_mode_init(uint8_t val){
	//关闭回显
	send_msg_ble("TTM:ECHO-0");
	//设置工作模式
	uint8_t msg[30];
	sprintf((char*)msg,"TTM:MODE-%d",val);
	send_msg_ble(msg);
	//模块重命名为test3123
	send_msg_ble("TTM:REN-test3123M");
	//关闭蓝牙广播
	send_msg_ble("TTM:ADV-0");
}


/*通过指定MAC地址连接远端蓝牙模块*/
void connet_remote_ble(void)
{
	uint8_t msg[128];
	sprintf((char*)msg,"TTM:CONN-%s",REMOTE_MAC);
	send_msg_ble(msg);
}

void discon_remote_ble(void)
{
	//主机最多同时连接5个从机，测试状态已知只连接水压检测上的蓝牙从站
	for(uint8_t idx=0;idx<1;idx++){
		uint8_t msg[64];
		sprintf((char*)msg,"TTM:MASTER-DISCONN-#%d",idx);
		send_msg_ble(msg);
	}
}

/*蓝牙主机向蓝牙从机发送信息(主控向水压检测蓝牙发送)*/
void send_remote_ble(uint8_t* msg){
	//测试期间直接向所有主机发送消息：Todo：可通过查询已连接用户表确定向哪个主机发送信息
	for(uint8_t idx=0;idx<1;idx++){
		uint8_t send_msg[128];
		sprintf((char*)send_msg,"TTM:MASTER-SEND-#%d,%s",idx,msg);
		send_msg_ble(send_msg);
	}
}

/*蓝牙从机向蓝牙主机发送信息(主控板蓝牙向用户主机发送状态信息)*/
void send_user_ble(uint8_t *msg){
	uint8_t send_msg[200];
	sprintf((char*)send_msg,"TTM:SLAVE-SEND-%s",msg);
	send_msg_ble(send_msg);
}

/* USER CODE END 1 */
