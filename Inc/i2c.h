/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "stm32l0xx_hal.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
	#define I2C_READ_BIT 		1
#define TIMEOUT_MS     50
#define LID3DH_POWER_UP_MS     10

#define LIS3DH_ADDR             0x19  // 7-bit I2C address. If SA0 is pulled high, address is 0x19.
#define LIS3DH_DEVICE_ID        0x33  // Contents of WHO_AM_I register.

/*由LIS3DH芯片手册表21定义寄存器地址*/
#define REG_STATUS_REG_AUX      0x07  // r
#define REG_OUT_ADC1_L          0x08  // r
#define REG_OUT_ADC1_H          0x09  // r
#define REG_OUT_ADC2_L          0x0a  // r
#define REG_OUT_ADC2_H          0x0b  // r
#define REG_OUT_ADC3_L          0x0c  // r
#define REG_OUT_ADC3_H          0x0d  // r
#define REG_WHO_AM_I            0x0f  // r
#define REG_CTRL_REG0           0x1e  // rw
#define REG_TEMP_CFG_REG        0x1f  // rw
#define REG_CTRL_REG1           0x20  // rw
#define REG_CTRL_REG2           0x21  // rw
#define REG_CTRL_REG3           0x22  // rw
#define REG_CTRL_REG4           0x23  // rw
#define REG_CTRL_REG5           0x24  // rw
#define REG_CTRL_REG6           0x25  // rw
#define REG_REFERENCE           0x26  // rw
#define REG_STATUS_REG          0x27  // r
#define REG_OUT_XYZ_BASE        0x28  // r  (Base for reading the six XYZ registers consecutively)
#define REG_OUT_X_L             0x28  // r
#define REG_OUT_X_H             0x29  // r
#define REG_OUT_Y_L             0x2a  // r
#define REG_OUT_Y_H             0x2b  // r
#define REG_OUT_Z_L             0x2c  // r
#define REG_OUT_Z_H             0x2d  // r
#define REG_FIFO_CTRL_REG       0x2e  // rw
#define REG_FIFO_SRC_REG        0x2f  // r
#define REG_INT1_CFG            0x30  // rw
#define REG_INT1_SRC            0x31  // r
#define REG_INT1_THS            0x32  // rw
#define REG_INT1_DURATION       0x33  // rw
#define REG_INT2_CFG            0x34  // rw
#define REG_INT2_SRC            0x35  // r
#define REG_INT2_THS            0x36  // rw
#define REG_INT2_DURATION       0x37  // rw
#define REG_CLICK_CFG           0x38  // rw
#define REG_CLICK_SRC           0x39  // r
#define REG_CLICK_THS           0x3a  // rw
#define REG_TIME_LIMIT          0x3b  // rw
#define REG_TIME_LATENCY        0x3c  // rw
#define REG_TIME_WINDOW         0x3d  // rw
#define REG_ACT_THS             0x3c  // rw
#define REG_ACT_DUR             0x3f  // rw

/*由芯片手册表31 ODR寄存器设定*/
#define DATA_RATE_POWER_DOWN	    0x0 // Power-down mode. Default.
#define DATA_RATE_LOW_1Hz       (0x1 << 4)
#define DATA_RATE_LOW_10Hz      (0x2 << 4)
#define DATA_RATE_LOW_25Hz      (0x3 << 4)
#define DATA_RATE_LOW_50Hz      (0x4 << 4)
#define DATA_RATE_LOW_100Hz     (0x5 << 4)
#define DATA_RATE_LOW_200Hz     (0x6 << 4)
#define DATA_RATE_LOW_400Hz     (0x7 << 4)
#define DATA_RATE_LOW_1kHz6     (0x8 << 4)
#define DATA_RATE_NORM_1kHz344  (0x9 << 4)
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct{
	void* i2c;	//IIC句柄
	uint16_t i2c_addr;	//7bitIIC地址

	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t t;
	/* Buffer for data read from the device. Must be 6 bytes to ready XYZ data. */
	uint16_t bufsize;
	uint8_t *buf;
} lis3dh_t;

extern lis3dh_t g_lis3dh;
extern uint8_t lis3dh_buffer[6];

HAL_StatusTypeDef lis3dh_init(lis3dh_t *lis3dh, I2C_HandleTypeDef *i2c, uint8_t *buf, uint16_t bufsize);
bool lis3dh_xyz_available(lis3dh_t *lis3dh);
HAL_StatusTypeDef lis3dh_read(lis3dh_t *lis3dh, uint16_t reg, uint16_t bufsize);
HAL_StatusTypeDef lis3dh_write(lis3dh_t *lis3dh, uint16_t reg, uint8_t data);
HAL_StatusTypeDef lis3dh_get_xyz(lis3dh_t *lis3dh);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

