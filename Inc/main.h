/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define USART_REC_LEN  	512  		//定义最大接收字节数 256
#define RXBUFFERSIZE   1 				//缓存大小

extern uint16_t USART1_RX_STA;      
extern uint8_t USART1_RX_BUF[USART_REC_LEN];
extern uint8_t USART1_TX_BUF[USART_REC_LEN];
extern uint8_t u1_aRxBuffer[RXBUFFERSIZE];

extern uint16_t USART2_RX_STA;      
extern uint8_t USART2_RX_BUF[USART_REC_LEN];
extern uint8_t USART2_TX_BUF[USART_REC_LEN];
extern uint8_t u2_aRxBuffer[RXBUFFERSIZE];

extern uint16_t LPUSART1_RX_STA;
extern uint8_t LPUSART1_RX_BUF[USART_REC_LEN];
extern uint8_t LPUSART1_TX_BUF[USART_REC_LEN]; 
extern uint8_t lp1_aRxBuffer[RXBUFFERSIZE];


extern uint8_t NB_4G_State;
extern uint8_t Water_State;
extern float Voleage;
extern uint8_t Water_Pre[10];
uint8_t process_remote_ble_recv(uint8_t *buffer);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
