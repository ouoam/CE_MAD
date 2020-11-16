/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "ff.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS USBH_fatfs;
FIL MyFile;
FRESULT res;
uint32_t bytesWritten;
uint8_t rtext[200];
uint8_t wtext[] = "USB Host Library: Mass Storage Example";
char name[10];
uint16_t counter = 0;
uint32_t ic = 0;
static int32_t uart_length = 0;
extern char USBHPath[];

extern UART_HandleTypeDef huart3;
char uart_tx_buffer[100];
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
void userFunction(uint8_t* pData, uint32_t pDataLength)
{
  uint16_t bytesread;
  if (Appli_state == APPLICATION_READY)
  {
    // if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    // else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    // if ((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) && ic > 0xfffff)
    // {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      ic = 0;
      //File name creation
      sprintf(name, "%d.jpg", counter++);
      /*Create a file*/
      if (f_open(&MyFile, name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
        /* Creation failed */
        // uart_length = sprintf(uart_tx_buffer, "Cannot open %s file \r\n", name);
        // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
      }
      else
      {
        // uart_length = sprintf(uart_tx_buffer, "file %s created \r\n", name);
        // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
        /*write message to the file. Use variable wtext, bytesWritten*/
        res = f_write(&MyFile, pData, pDataLength, &bytesWritten);

        /*close the file*/
        f_close(&MyFile);

        /*check number of written bytes*/
        if ((bytesWritten == 0) || (res != FR_OK))
        {
          /*error during writing*/
          // uart_length = sprintf(uart_tx_buffer, "write error \r\n");
          // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        }
        else
        {
          /*open file to verification*/
          if (f_open(&MyFile, name, FA_READ) != FR_OK)
          {
            /*file open failure*/
            // uart_length = sprintf(uart_tx_buffer, "Cannot open %s file for verify \r\n", name);
            // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
          }
          else
          {
            /*Read file content. Use variable : rtext, bytesread*/
            res = f_read(&MyFile, rtext, sizeof(rtext),&bytesread);

            if ((bytesread == 0) || (res != FR_OK))
            {
              /*read fail*/
              // uart_length = sprintf(uart_tx_buffer, "Cannot read file for verification \r\n");
              // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            }
            else
            {
              /*read success*/
            }

            /*close the file*/
            if (f_close(&MyFile) != FR_OK)
            {
              /*check number of written bytes*/
              // uart_length = sprintf(uart_tx_buffer, "fclose fail \r\n");
              // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
              while (1)
              {
                HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
              }
            }
          }
          /* Compare read data with the expected data */
          if ((bytesread == bytesWritten))
          {
            /*verification success full - number of written bytes is equal to number of read bytes*/
            // uart_length = sprintf(uart_tx_buffer, "verification OK - read number of bytes is equal to written number of bytes \r\n");
            // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, ((uint16_t)uart_length), 5000);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
          }
          else
          {
            /*verification failed - number of written bytes is not equal to number of read bytes*/
            // uart_length = sprintf(uart_tx_buffer, "verify fail \r\n");
            // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
          }
          /*end program execution after verification*/
        }
      }
    // }
    // ic++;
  }
}
/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  // uart_length = sprintf(uart_tx_buffer, "application ready\r\n");
  // HAL_UART_Transmit(&huart3,(uint8_t *)uart_tx_buffer, (uint16_t)uart_length, 1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  if (f_mount(&USBH_fatfs, USBHPath, 0) != FR_OK)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
