/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "jpeg_utils.h"
#include "encode_dma.h"

//#include "ov7670.h"
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
void RGB_GetInfo(JPEG_ConfTypeDef *pInfo);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define CAM_RESET_Pin GPIO_PIN_0
#define CAM_RESET_GPIO_Port GPIOG
#define CAM_PWDN_Pin GPIO_PIN_1
#define CAM_PWDN_GPIO_Port GPIOG
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define RES_VGA_W     640
#define RES_VGA_H     480
#define RES_QVGA_W    ( RES_VGA_W / 2 )   // 320
#define RES_QVGA_H    ( RES_VGA_H / 2 )   // 240
#define RES_QQVGA_W   ( RES_QVGA_W / 2 )  // 160
#define RES_QQVGA_H   ( RES_QVGA_H / 2 )  // 120

#define FRAME_SIZE        (FRAMESIZE_QVGA)
#define FRAME_SIZE_WIDTH  (RES_QVGA_W)
#define FRAME_SIZE_HEIGHT (RES_QVGA_H)

#define JPEG_CHROMA_SAMPLING     JPEG_422_SUBSAMPLING   /* Select Chroma Sampling: JPEG_420_SUBSAMPLING, JPEG_422_SUBSAMPLING, JPEG_444_SUBSAMPLING   */
#define JPEG_COLOR_SPACE         JPEG_YCBCR_COLORSPACE  /* Select Color Space: JPEG_YCBCR_COLORSPACE, JPEG_GRAYSCALE_COLORSPACE, JPEG_CMYK_COLORSPACE */
#define JPEG_IMAGE_QUALITY       75                     /* Set Image Quality for Jpeg Encoding */
#define MAX_INPUT_WIDTH          800                    /* Set the Maximum of RGB input images Width to be encoded */
#define MAX_INPUT_LINES          8                      /* Set Input buffer lines to 16 for YCbCr420, and 8 for YCbCr422 and YCbCr444 (to save RAM space) */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
