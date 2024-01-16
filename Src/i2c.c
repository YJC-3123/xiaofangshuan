/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */
lis3dh_t g_lis3dh;
uint8_t lis3dh_buffer[6];
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef lis3dh_init(lis3dh_t *lis3dh, I2C_HandleTypeDef *i2c, uint8_t *buf, uint16_t bufsize) {
	HAL_StatusTypeDef status;

	lis3dh->i2c = i2c;
	lis3dh->i2c_addr = LIS3DH_ADDR << 1;
	lis3dh->buf = buf;
	lis3dh->bufsize = bufsize;

	HAL_Delay(LID3DH_POWER_UP_MS);
	status = HAL_I2C_IsDeviceReady(lis3dh->i2c, lis3dh->i2c_addr, 1, TIMEOUT_MS);
	if (status != HAL_OK) return status;

  //LIS3DH自识别 
	status = lis3dh_read(lis3dh, REG_WHO_AM_I, 1);
	if (status != HAL_OK) return status;
	if (lis3dh->buf[0] != LIS3DH_DEVICE_ID) return HAL_ERROR;

	//设置数据刷新速率
	status = lis3dh_write(lis3dh, REG_CTRL_REG1, DATA_RATE_NORM_1kHz344 | 0x07);
	if (status != HAL_OK) return status;

	//设置DBU、HR
	status = lis3dh_write(lis3dh, REG_CTRL_REG4, 0x88);
	if (status != HAL_OK) return status;

	//使能温度传感器
	//status = lis3dh_write(lis3dh, REG_TEMP_CFG_REG, 0x80);
	return status;
}

#include "string.h"
#include "stdio.h"
extern UART_HandleTypeDef huart1;
bool lis3dh_xyz_available(lis3dh_t *lis3dh) {
	/*
	 * Read STATUS_REG bit 2 (ZYXDA): New X, Y, Z data available.
	 */
	HAL_StatusTypeDef status;
	status = lis3dh_read(lis3dh, REG_STATUS_REG, 1);
	
	if (status != HAL_OK) 
		return false;
	return (lis3dh->buf[0] & 2) > 0;
}

HAL_StatusTypeDef lis3dh_read(lis3dh_t* lis3dh, uint16_t reg, uint16_t bufsize) {
	//读一个LIS3DHTR的8位寄存器
	if (bufsize > lis3dh->bufsize) 
		return HAL_ERROR;
	return HAL_I2C_Mem_Read(lis3dh->i2c, lis3dh->i2c_addr | I2C_READ_BIT, reg, 1, lis3dh->buf, bufsize, TIMEOUT_MS);
}

HAL_StatusTypeDef lis3dh_write(lis3dh_t* lis3dh, uint16_t reg, uint8_t data) {
	//写一个LIS3DHTR的8位寄存器
	return HAL_I2C_Mem_Write(lis3dh->i2c, lis3dh->i2c_addr, reg, 1, &data, 1, TIMEOUT_MS);
}

HAL_StatusTypeDef lis3dh_get_xyz(lis3dh_t* lis3dh) {
	if (lis3dh->bufsize < 6) return HAL_ERROR;
		
	//从XYZ输出寄存器基地址开始读6个byte对应XYZ和L和H
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
			lis3dh->i2c,
			lis3dh->i2c_addr | I2C_READ_BIT,
			REG_OUT_XYZ_BASE | 0x80,
			1,
			lis3dh->buf,
			6,
			TIMEOUT_MS);

	if (status != HAL_OK) {
		lis3dh->x = 0;
		lis3dh->y = 0;
		lis3dh->z = 0;
		return status;
	}

	lis3dh->x = (int) (((int8_t) lis3dh->buf[1]) << 8) | lis3dh->buf[0];
	lis3dh->y = (int) (((int8_t) lis3dh->buf[3]) << 8) | lis3dh->buf[2];
	lis3dh->z = (int) (((int8_t) lis3dh->buf[5]) << 8) | lis3dh->buf[4];

	return HAL_OK;
}
/* USER CODE END 1 */
