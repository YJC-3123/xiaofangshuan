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
void print_u1(uint8_t *msg);
void GNSS_data_parse(uint8_t * buff_t);
void nb_module_init(void);
void cat1_module_init(void);
void check_cat1_recvmsg(uint8_t* recvmsg);
void send_msg_nbtcp_server(uint8_t* msg);
void send_msg_cat1_tcp_server(uint8_t* msg);
HAL_StatusTypeDef connect_tcp_server(void);

void send_msg_ble(uint8_t *msg);
void ble_mode_init(uint8_t val);
void connet_remote_ble(void);
void discon_remote_ble(void);
void send_remote_ble(uint8_t* msg);
void send_user_ble(uint8_t *msg);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

