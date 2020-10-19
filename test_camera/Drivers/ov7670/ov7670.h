/*
 * ov7670.h
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 */

#ifndef OV7670_OV7670_H_
#define OV7670_OV7670_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "ov7670Reg.h"

#define OV7670_MODE_QVGA_RGB565 0
#define OV7670_MODE_QVGA_YUV    1

#define OV7670_CAP_CONTINUOUS   0
#define OV7670_CAP_SINGLE_FRAME 1

#define RES_VGA_W			640
#define RES_VGA_H			480
#define RES_QVGA_W		( RES_VGA_W / 2 )		// 320
#define RES_QVGA_H		( RES_VGA_H / 2 )		// 240
#define RES_QQVGA_W		( RES_QVGA_W / 2 )	// 160
#define RES_QQVGA_H		( RES_QVGA_H / 2 )	// 120

#define RES_CIF_W			352
#define RES_CIF_H			288
#define RES_QCIF_W		( RES_CIF_W / 2 )		// 176
#define RES_QCIF_H		( RES_CIF_H / 2 )		// 144
#define RES_QQCIF_W		( RES_QCIF_W / 2 )	// 88
#define RES_QQCIF_H		( RES_QCIF_H / 2 )	// 72

HAL_StatusTypeDef ov7670_init(DCMI_HandleTypeDef *p_hdcmi, I2C_HandleTypeDef *p_hi2c);
HAL_StatusTypeDef ov7670_startCap(uint32_t capMode, uint32_t destAddress);
HAL_StatusTypeDef ov7670_stopCap();
void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));

#endif /* OV7670_OV7670_H_ */
