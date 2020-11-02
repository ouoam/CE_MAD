/**
  ******************************************************************************
  * File Name          : JPEG.c
  * Description        : This file provides code for the configuration
  *                      of the JPEG instances.
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

/* Includes ------------------------------------------------------------------*/
#include "jpeg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

JPEG_HandleTypeDef hjpeg;
DMA_HandleTypeDef hdma_jpeg_in;
DMA_HandleTypeDef hdma_jpeg_out;

/* JPEG init function */
void MX_JPEG_Init(void)
{

  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_JPEG_MspInit(JPEG_HandleTypeDef* jpegHandle)
{

  if(jpegHandle->Instance==JPEG)
  {
  /* USER CODE BEGIN JPEG_MspInit 0 */

  /* USER CODE END JPEG_MspInit 0 */
    /* JPEG clock enable */
    __HAL_RCC_JPEG_CLK_ENABLE();

    /* JPEG DMA Init */
    /* JPEG_IN Init */
    hdma_jpeg_in.Instance = DMA2_Stream0;
    hdma_jpeg_in.Init.Channel = DMA_CHANNEL_9;
    hdma_jpeg_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_jpeg_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_jpeg_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_jpeg_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_jpeg_in.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_jpeg_in.Init.Mode = DMA_NORMAL;
    hdma_jpeg_in.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_jpeg_in.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_jpeg_in.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_jpeg_in.Init.MemBurst = DMA_MBURST_INC4;
    hdma_jpeg_in.Init.PeriphBurst = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_jpeg_in) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(jpegHandle,hdmain,hdma_jpeg_in);

    /* JPEG_OUT Init */
    hdma_jpeg_out.Instance = DMA2_Stream1;
    hdma_jpeg_out.Init.Channel = DMA_CHANNEL_9;
    hdma_jpeg_out.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_jpeg_out.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_jpeg_out.Init.MemInc = DMA_MINC_ENABLE;
    hdma_jpeg_out.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_jpeg_out.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_jpeg_out.Init.Mode = DMA_NORMAL;
    hdma_jpeg_out.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_jpeg_out.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_jpeg_out.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_jpeg_out.Init.MemBurst = DMA_MBURST_INC4;
    hdma_jpeg_out.Init.PeriphBurst = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&hdma_jpeg_out) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(jpegHandle,hdmaout,hdma_jpeg_out);

    /* JPEG interrupt Init */
    HAL_NVIC_SetPriority(JPEG_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(JPEG_IRQn);
  /* USER CODE BEGIN JPEG_MspInit 1 */

  /* USER CODE END JPEG_MspInit 1 */
  }
}

void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef* jpegHandle)
{

  if(jpegHandle->Instance==JPEG)
  {
  /* USER CODE BEGIN JPEG_MspDeInit 0 */

  /* USER CODE END JPEG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_JPEG_CLK_DISABLE();

    /* JPEG DMA DeInit */
    HAL_DMA_DeInit(jpegHandle->hdmain);
    HAL_DMA_DeInit(jpegHandle->hdmaout);

    /* JPEG interrupt Deinit */
    HAL_NVIC_DisableIRQ(JPEG_IRQn);
  /* USER CODE BEGIN JPEG_MspDeInit 1 */

  /* USER CODE END JPEG_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
