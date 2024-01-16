/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef hlpuart1;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
typedef struct {
	double Lon;     //经纬度
	double Lat;
  char Lon_area;	//经纬度方向
  char Lat_area;
  uint8_t Time_H;   //时间
  uint8_t Time_M;
  uint8_t Time_S;
  uint8_t Status;   //定位状态
} GNRMC;


/* USER CODE END Private defines */

void MX_LPUART1_UART_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART_IT_Start(void);
void GNSS_data_parse(uint8_t * buff_t);

HAL_StatusTypeDef connect_ble(void);
void send_msg_ble(uint8_t* msg);
void print_u1(uint8_t *msg);
void nb_module_init(void);
void send_msg_tcp_server(uint8_t* msg);
HAL_StatusTypeDef connect_tcp_server(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

