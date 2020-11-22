/**
  ******************************************************************************
  * @file    JPEG/JPEG_EncodingFromFLASH_DMA/Src/encode_dma.c
  * @author  MCD Application Team
  * @brief   This file provides routines for JPEG Encoding from memory with
  *          DMA method.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
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
#include "encode_dma.h"
/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup JPEG_EncodingFromFLASH_DMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t State;
  uint8_t *DataBuffer;
  uint32_t DataBufferSize;

}JPEG_Data_BufferTypeDef;

/* Private define ------------------------------------------------------------*/
#if (JPEG_RGB_FORMAT == JPEG_ARGB8888)
#define BYTES_PER_PIXEL    4
#elif (JPEG_RGB_FORMAT == JPEG_RGB888)
#define BYTES_PER_PIXEL    3
#elif (JPEG_RGB_FORMAT == JPEG_RGB565)
#define BYTES_PER_PIXEL    2
#endif

#define CHUNK_SIZE_IN   ((uint32_t)(MAX_INPUT_WIDTH * BYTES_PER_PIXEL * MAX_INPUT_LINES))
#define CHUNK_SIZE_OUT  ((uint32_t) (1024))

#define JPEG_BUFFER_EMPTY       0
#define JPEG_BUFFER_FULL        1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// JPEG_RGBToYCbCr_Convert_Function pRGBToYCbCr_Convert_Function;

uint8_t MCU_Data_IntBuffer0[CHUNK_SIZE_IN];

uint8_t JPEG_Data_OutBuffer0[CHUNK_SIZE_OUT];

JPEG_Data_BufferTypeDef Jpeg_OUT_BufferTab = {JPEG_BUFFER_EMPTY , JPEG_Data_OutBuffer0 , 0};

JPEG_Data_BufferTypeDef Jpeg_IN_BufferTab = {JPEG_BUFFER_EMPTY , MCU_Data_IntBuffer0, 0};

uint32_t MCU_TotalNb                = 0;
uint32_t MCU_BlockIndex             = 0;
uint32_t Jpeg_HWEncodingEnd         = 0;


__IO uint32_t Output_Is_Paused      = 0;
__IO uint32_t Input_Is_Paused       = 0;

JPEG_ConfTypeDef Conf;
UART_HandleTypeDef *pHuart;
JPEG_HandleTypeDef *pJpeg;

uint32_t RGB_InputImageIndex;
uint32_t RGB_InputImageSize_Bytes;
uint8_t *RGB_InputImageAddress;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Encode_DMA
  * @param hjpeg: JPEG handle pointer
  * @param  FileName    : jpg file path for decode.
  * @param  DestAddress : ARGB destination Frame Buffer Address.
  * @retval None
  */
uint32_t JPEG_Encode_DMA(JPEG_HandleTypeDef *hjpeg, uint8_t *RGBImageBufferAddress, uint32_t RGBImageSize_Bytes, UART_HandleTypeDef *huart)
{
  pHuart = huart;
  pJpeg = hjpeg;
  uint32_t DataBufferSize = 0;

  /* Reset all Global variables */
  MCU_TotalNb                = 0;
  MCU_BlockIndex             = 0;
  Jpeg_HWEncodingEnd         = 0;
  Output_Is_Paused           = 0;
  Input_Is_Paused            = 0;

  /* Get RGB Info */
  RGB_GetInfo(&Conf);
  // JPEG_GetEncodeColorConvertFunc(&Conf, &pRGBToYCbCr_Convert_Function, &MCU_TotalNb);

  uint32_t hMCU = (Conf.ImageWidth / 16);
  uint32_t vMCU = (Conf.ImageHeight / 8);

  MCU_TotalNb = (hMCU * vMCU);

  /* Clear Output Buffer */
  Jpeg_OUT_BufferTab.DataBufferSize = 0;
  Jpeg_OUT_BufferTab.State = JPEG_BUFFER_EMPTY;

  /* Fill input Buffers */
  RGB_InputImageIndex = 0;
  RGB_InputImageAddress = RGBImageBufferAddress;
  RGB_InputImageSize_Bytes = RGBImageSize_Bytes;
  DataBufferSize = Conf.ImageWidth * MAX_INPUT_LINES * BYTES_PER_PIXEL;

  if(RGB_InputImageIndex < RGB_InputImageSize_Bytes)
  {
    /* Pre-Processing */
    //MCU_BlockIndex += pRGBToYCbCr_Convert_Function((uint8_t *)(RGB_InputImageAddress + RGB_InputImageIndex), Jpeg_IN_BufferTab.DataBuffer, 0, DataBufferSize,(uint32_t*)(&Jpeg_IN_BufferTab.DataBufferSize));

    MCU_BlockIndex += Conf.ImageWidth / 16;
    Jpeg_IN_BufferTab.DataBufferSize = Conf.ImageWidth * MAX_INPUT_LINES * BYTES_PER_PIXEL;
//    for (int i = 0; i < DataBufferSize; i++) {
//      Jpeg_IN_BufferTab.DataBuffer[i] = RGB_InputImageAddress[i];
//    }
    Jpeg_IN_BufferTab.DataBuffer = RGB_InputImageAddress;

    Jpeg_IN_BufferTab.State = JPEG_BUFFER_FULL;

    RGB_InputImageIndex += DataBufferSize;
  }

  /* Fill Encoding Params */
  HAL_JPEG_ConfigEncoding(hjpeg, &Conf);

  /* Start JPEG encoding with DMA method */
  HAL_JPEG_Encode_DMA(hjpeg, Jpeg_IN_BufferTab.DataBuffer, Jpeg_IN_BufferTab.DataBufferSize, Jpeg_OUT_BufferTab.DataBuffer, CHUNK_SIZE_OUT);

  return 0;
}

/**
  * @brief JPEG Ouput Data BackGround processing .
  * @param hjpeg: JPEG handle pointer
  * @retval 1 : if JPEG processing has finiched, 0 : if JPEG processing still ongoing
  */
uint8_t JPEG_EncodeOutputHandler(JPEG_HandleTypeDef *hjpeg)
{
  if(Jpeg_HWEncodingEnd != 0)
  {
    while (pHuart->gState != HAL_UART_STATE_READY);
    return 1;
  }

  return 0;
}

/**
  * @brief JPEG Input Data BackGround Preprocessing .
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
uint8_t JPEG_EncodeInputHandler(JPEG_HandleTypeDef *hjpeg)
{
  uint32_t DataBufferSize = Conf.ImageWidth * MAX_INPUT_LINES * BYTES_PER_PIXEL;

  //if((Jpeg_IN_BufferTab.State == JPEG_BUFFER_EMPTY) && (MCU_BlockIndex <= MCU_TotalNb))
  {
    /* Read and reorder lines from RGB input and fill data buffer */
    if(RGB_InputImageIndex < RGB_InputImageSize_Bytes)
    {
      /* Pre-Processing */
      //MCU_BlockIndex += pRGBToYCbCr_Convert_Function((uint8_t *)(RGB_InputImageAddress + RGB_InputImageIndex), Jpeg_IN_BufferTab.DataBuffer, 0, DataBufferSize, (uint32_t*)(&Jpeg_IN_BufferTab.DataBufferSize));

      MCU_BlockIndex += Conf.ImageWidth / 16;
      Jpeg_IN_BufferTab.DataBufferSize = Conf.ImageWidth * MAX_INPUT_LINES * BYTES_PER_PIXEL;
//      for (int i = 0; i < DataBufferSize; i++) {
//        Jpeg_IN_BufferTab.DataBuffer[i] = RGB_InputImageAddress[i];
//      }
      Jpeg_IN_BufferTab.DataBuffer = RGB_InputImageAddress;

      Jpeg_IN_BufferTab.State = JPEG_BUFFER_FULL;
      RGB_InputImageIndex += DataBufferSize;

      if(Input_Is_Paused == 1)
      {
        Input_Is_Paused = 0;
        HAL_JPEG_ConfigInputBuffer(hjpeg, Jpeg_IN_BufferTab.DataBuffer, Jpeg_IN_BufferTab.DataBufferSize);

        HAL_JPEG_Resume(hjpeg, JPEG_PAUSE_RESUME_INPUT);
      }
    }
    else
    {
      MCU_BlockIndex++;
    }
    return 1;
  }
//  else {
//    return 0;
//  }
}

/**
  * @brief JPEG Get Data callback
  * @param hjpeg: JPEG handle pointer
  * @param NbEncodedData: Number of encoded (consummed) bytes from input buffer
  * @retval None
  */
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbEncodedData)
{
  if(NbEncodedData == Jpeg_IN_BufferTab.DataBufferSize)
  {
    Jpeg_IN_BufferTab.State = JPEG_BUFFER_EMPTY;
    Jpeg_IN_BufferTab.DataBufferSize = 0;

    HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
    Input_Is_Paused = 1;
  }
  else
  {
    HAL_JPEG_ConfigInputBuffer(hjpeg, Jpeg_IN_BufferTab.DataBuffer + NbEncodedData, Jpeg_IN_BufferTab.DataBufferSize - NbEncodedData);
  }
}

/**
  * @brief JPEG Data Ready callback
  * @param hjpeg: JPEG handle pointer
  * @param pDataOut: pointer to the output data buffer
  * @param OutDataLength: length of output buffer in bytes
  * @retval None
  */
void HAL_JPEG_DataReadyCallback (JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
  Jpeg_OUT_BufferTab.State = JPEG_BUFFER_FULL;
  Jpeg_OUT_BufferTab.DataBufferSize = OutDataLength;

  HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
  Output_Is_Paused = 1;

  HAL_JPEG_ConfigOutputBuffer(hjpeg, Jpeg_OUT_BufferTab.DataBuffer, CHUNK_SIZE_OUT);

  if(HAL_UART_Transmit_DMA(pHuart, Jpeg_OUT_BufferTab.DataBuffer, Jpeg_OUT_BufferTab.DataBufferSize)!= HAL_OK)
  {
    NVIC_SystemReset();
    // Transfer error in transmission process
    Error_Handler();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(Jpeg_OUT_BufferTab.State == JPEG_BUFFER_FULL)
  {
    Jpeg_OUT_BufferTab.State = JPEG_BUFFER_EMPTY;
    Jpeg_OUT_BufferTab.DataBufferSize = 0;

    if(Output_Is_Paused == 1)
    {
      Output_Is_Paused = 0;
      HAL_JPEG_Resume(pJpeg, JPEG_PAUSE_RESUME_OUTPUT);
    }
  }

  if(Jpeg_HWEncodingEnd != 0)
  {
    HAL_UART_Transmit(huart, (uint8_t*)"*RDY*", 5, 100);
  }
}

/**
  * @brief  JPEG Error callback
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
  Error_Handler();
}

/*
  * @brief JPEG Decode complete callback
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
  Jpeg_HWEncodingEnd = 1;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
