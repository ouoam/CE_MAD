/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "jpeg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* PRINTF REDIRECT to UART BEGIN */
// @see    http://www.keil.com/forum/60531/
// @see    https://stackoverflow.com/questions/45535126/stm32-printf-redirect

FILE __stdout;

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

int ferror(FILE *f){
  /* Your implementation of ferror(). */
  return 0;
}
/* PRINTF REDIRECT to UART END */

uint8_t buffCAM[MAX_INPUT_LINES][FRAME_SIZE_WIDTH * 2];

int aa = 0;
int bb = 0;

uint32_t last;

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	uint32_t now = HAL_GetTick();
  //printf("FRAME %d\n", HAL_GetTick());
	//printf("%d %d %f\r\n", aa, bb, 1000.0 / (now  - last));
	// printf("*RDY*");
	HAL_UART_Transmit(&huart3, (uint8_t*)"*RDY*", 5, 100);
	aa = 0;
	bb = 0;
	last = now;
	//HAL_UART_Transmit(&huart3, (uint8_t *)buffCAM, FRAME_SIZE_WIDTH * FRAME_SIZE_HEIGHT * 2, 1000);
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  //printf("VSYNC %d\n", HAL_GetTick());
	bb++;
}

#define CHUNK_SIZE_IN1   ((uint32_t)(FRAME_SIZE_WIDTH * 2 * MAX_INPUT_LINES))
#define MCU_PER_BUFF    ((uint32_t)(FRAME_SIZE_WIDTH * 2 * MAX_INPUT_LINES) / (4 * 8 * 8))
uint8_t MCU_Data_IntBuffer1[MCU_PER_BUFF][4][8][8];

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	//printf("HSYNC %d\n", HAL_GetTick());
	//HAL_UART_Transmit(&huart3, (uint8_t*)buffCAM[aa%MAX_INPUT_LINES], FRAME_SIZE_WIDTH * 2, 1000);
	while (huart3.gState != HAL_UART_STATE_READY);
	if(HAL_UART_Transmit_DMA(&huart3, buffCAM[aa%MAX_INPUT_LINES], FRAME_SIZE_WIDTH * 2)!= HAL_OK)
	{
		NVIC_SystemReset();
		/* Transfer error in transmission process */
		Error_Handler();
	}

  uint8_t buffLine = aa%MAX_INPUT_LINES;

  for (int j = 0; j < FRAME_SIZE_WIDTH * 2; j+=4) {
    uint32_t MCUindex = j / (8 * 2 * 2);
    uint32_t MCUindexBlock = (j/(8*2)) % 2;
    MCU_Data_IntBuffer1[MCUindex][MCUindexBlock][buffLine][(j/2) % 8]   = buffCAM[buffLine][j + 0];
    MCU_Data_IntBuffer1[MCUindex][MCUindexBlock][buffLine][(j/2+1) % 8] = buffCAM[buffLine][j + 2];

    MCU_Data_IntBuffer1[MCUindex][2][buffLine][(j/4) % 8] = buffCAM[buffLine][j + 1];
    MCU_Data_IntBuffer1[MCUindex][3][buffLine][(j/4) % 8] = buffCAM[buffLine][j + 3];
  }

  if (aa == 7) {
    // JPEG_Encode_DMA(&hjpeg, )
  } else if (aa % 8 == 7) {

  }

	aa++;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_I2S3_Init();
  MX_JPEG_Init();
  /* USER CODE BEGIN 2 */
	//ov7670_init(&hdcmi, &hi2c2);
	
	HAL_Delay(100);

	//HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&buffCAM, MAX_INPUT_LINES * FRAME_SIZE_WIDTH * 2 / 4);
	
  /* USER CODE END 2 */

	uint8_t round = 0;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (int i = 0; i < FRAME_SIZE_HEIGHT; i++) {
      for (int j = 0; j < FRAME_SIZE_WIDTH * 2; j+=4) {
    	  /*
		  switch (j / (FRAME_SIZE_WIDTH / 2 /5)) {
		  case 0: buffCAM[i % MAX_INPUT_LINES][j] = 0x007F007F; break;
		  case 1: buffCAM[i % MAX_INPUT_LINES][j] = 0x7F7F007F; break;
		  case 2: buffCAM[i % MAX_INPUT_LINES][j] = 0xFF7F007F; break;
		  case 3: buffCAM[i % MAX_INPUT_LINES][j] = 0x007F7F7F; break;
		  case 4: buffCAM[i % MAX_INPUT_LINES][j] = 0x007FFF7F; break;
		  }
		  */
/*
      buffCAM[i % MAX_INPUT_LINES][j] = round << 16 | round |
                                  (uint8_t)(i * 255 / (FRAME_SIZE_HEIGHT-1)) << 24 |
                                  (uint8_t)(j * 255 / (FRAME_SIZE_WIDTH/2-1)) << 8;
      */

      buffCAM[i % MAX_INPUT_LINES][j + 0] = round;
        buffCAM[i % MAX_INPUT_LINES][j + 1] = j * 255 / (FRAME_SIZE_WIDTH * 2-1);
      buffCAM[i % MAX_INPUT_LINES][j + 2] = round;
      buffCAM[i % MAX_INPUT_LINES][j + 3] = i * 255 / (FRAME_SIZE_HEIGHT-1);

      }
      HAL_DCMI_LineEventCallback(&hdcmi);
      HAL_Delay(1000/FRAME_SIZE_HEIGHT/2);
    }
    HAL_DCMI_FrameEventCallback(&hdcmi);
    round+=8;
    //HAL_Delay(5000);
		// printf("1234567890\r\n");
		// HAL_UART_Transmit(&huart3, (unsigned char *)"1234567890\r\n", 12, 500);
		// HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 144;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 3;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sClockSelection = RCC_I2SCLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_4);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Get the images sizes from BMP header.
  * @param  pInfo : pointer to the Info structure
  * @retval None
  */
void RGB_GetInfo(JPEG_ConfTypeDef *pInfo)
{
  /* Read Images Sizes */
  pInfo->ImageWidth         = FRAME_SIZE_WIDTH;
  pInfo->ImageHeight        = FRAME_SIZE_HEIGHT;

  /* Jpeg Encoding Setting to be set by users */
  pInfo->ChromaSubsampling  = JPEG_CHROMA_SAMPLING;
  pInfo->ColorSpace         = JPEG_COLOR_SPACE;
  pInfo->ImageQuality       = JPEG_IMAGE_QUALITY;

  /*Check if Image Sizes meets the requirements */
  if (((pInfo->ImageWidth % 8) != 0 ) || ((pInfo->ImageHeight % 8) != 0 ) || \
      (((pInfo->ImageWidth % 16) != 0 ) && (pInfo->ColorSpace == JPEG_YCBCR_COLORSPACE) && (pInfo->ChromaSubsampling != JPEG_444_SUBSAMPLING)) || \
      (((pInfo->ImageHeight % 16) != 0 ) && (pInfo->ColorSpace == JPEG_YCBCR_COLORSPACE) && (pInfo->ChromaSubsampling == JPEG_420_SUBSAMPLING)))
  {
    Error_Handler();
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
