/**
  ******************************************************************************
  * @file    LwIP/LwIP_StreamingServer/Src/rtp_protocol.c 
  * @author  MCD Application Team
  * @brief   RTP client module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "rtp_protocol.h"
#include "encode_dma.h"

#include "ov7670.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RTP_HandleTypeDef RTP_struct;         /* RTP structure */
osSemaphoreId Sending_Semaphore;      /* Semaphore ID to signal transfer frame complete */
osThreadId Thr_Send_Sem;              /* Thread ID */
osThreadId simDCMItaskHandle;
volatile osThreadId thr_ID;           /* The thread ID */
const char jpegRTP_header[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x4B, 0x14, 0x0f}; // q, w/8, h/8

void StartSimDCMItask(void const * argument);
/* Imported variables---------------------------------------------------------*/
extern uint8_t JPEG_buffer[JPEG_BUFFER_SIZE]; /* JPEG buffer */
extern osSemaphoreId RTP_SendSemaphoreHandle;
extern uint32_t JPEG_ImgSize;

extern uint8_t buffCAM[MAX_INPUT_LINES * FRAME_SIZE_WIDTH * 2];
extern DCMI_HandleTypeDef hdcmi;
extern I2C_HandleTypeDef hi2c2;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize RTP thread 
  * @param  None
  * @retval None
  */
void RTP_Init(void)
{
  printf("RTP_Init\r\n");
  /* Reset the RTP structure */
  memset(&RTP_struct, 0x00, sizeof(RTP_struct));
  
  /* RTP state idle */
  RTP_struct.State = RTP_STATE_IDLE;
  
  /* Create new socket */
  RTP_struct.sock_id = socket(AF_INET, SOCK_DGRAM, 0);
  if (RTP_struct.sock_id >= 0)
  {
    /* Prepare local address */
    memset(&RTP_struct.local, 0x00, sizeof(RTP_struct.local));
    RTP_struct.local.sin_family = AF_INET;
    RTP_struct.local.sin_port = PP_HTONS(RTP_PORT);
    RTP_struct.local.sin_addr.s_addr = PP_HTONL(INADDR_ANY);
    
    /* Bind to local address */
    if (bind(RTP_struct.sock_id, (struct sockaddr *)&RTP_struct.local, sizeof(RTP_struct.local)) == 0) 
    {
      printf("RTP_Init bind\r\n");
      /* prepare RTP stream address */
      memset(&RTP_struct.net_dest, 0x00, sizeof(RTP_struct.net_dest));
      RTP_struct.net_dest.sin_family = AF_INET;
      RTP_struct.net_dest.sin_port = PP_HTONS(RTP_PORT);   
      RTP_struct.net_dest.sin_addr.s_addr = TARGET_IP_ADDRESS;
      
      /* Reset rtp packet */
      memset(RTP_struct.rtp_send_packet, 0x00, sizeof(RTP_struct.rtp_send_packet));
      
      osThreadDef(Snd, Send_Sem, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 4);
      Thr_Send_Sem = osThreadCreate(osThread(Snd), NULL);
      
      /*Start the continuous mode of the camera */
      // BSP_CAMERA_ContinuousStart((uint8_t *)RGB_buffer);
      /* definition and creation of simDCMItask */
//      osThreadDef(simDCMItask, StartSimDCMItask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 4);
//      simDCMItaskHandle = osThreadCreate(osThread(simDCMItask), NULL);

      ov7670_init(&hdcmi, &hi2c2);
      osDelay(100);
      HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)&buffCAM, MAX_INPUT_LINES * FRAME_SIZE_WIDTH * 2 / 4);

      RTP_struct.rtp_data = (char *)JPEG_buffer;
      
      /* Next state is RTP_STATE_START */
      RTP_struct.State = RTP_STATE_START;
    }
    else
    {
      /* Binding local adress failed */
      RTP_struct.State = RTP_STATE_ERROR;
    }
  }

}

/**
  * @brief  Sending thread
  * @param  None
  * @retval None
  */
void Send_Sem(void const * arg)
{
  struct rtp_hdr_t* rtphdr;          /* RTP header */
  uint8_t offset[3];                 /* The offset in the RTP/JPEG header */
  uint8_t* rtp_payload;              /* RTP payload */
  int rtp_payload_size = 0;          /* RTP payload size in the current packet */
  
  int sock_id = RTP_struct.sock_id;
  struct sockaddr_in* net_dest = &RTP_struct.net_dest;

  /* prepare RTP packet */
  rtphdr = (struct rtp_hdr_t*) RTP_struct.rtp_send_packet;
  rtphdr->version = RTP_VERSION;
  rtphdr->pt = 0;
  rtphdr->ssrc = PP_HTONL(0); 

  /* Set a payload pointer */
  rtp_payload = RTP_struct.rtp_send_packet + sizeof(struct rtp_hdr_t);

  /* set a RTP/JPEG header*/
  memcpy(rtp_payload, jpegRTP_header ,1 * sizeof(char));
  memcpy(rtp_payload + 4, jpegRTP_header + 4, 4 * sizeof(char));

  while(1)
  {
    // ((rtp_data_index + rtp_payload_size)  <= JPEG_ImgSize) || (rtphdr->pt != 0x9A)
    uint32_t val = 0;
    if( xTaskNotifyWait( 0, 0, &val, 1000 ) != 0 )
    {
      /* Increment the timestamp value */
      rtphdr->ts = htonl(ntohl(rtphdr->ts) + RTP_TIMESTAMP);

      /* Set a payload size */
      rtp_payload_size = min(RTP_PAYLOAD_SIZE_MAX, val);

      /* set a RTP/JPEG header*/
      offset[0]= (RTP_struct.Offset & 0x00FF0000) >> 16;
      offset[1]= (RTP_struct.Offset & 0x0000FF00) >> 8;
      offset[2]= RTP_struct.Offset & 0x000000FF;
      memcpy(rtp_payload + 1, offset, 3 * sizeof(char));

      /* Set a payload */
      memcpy(rtp_payload + 8, RTP_struct.rtp_data, rtp_payload_size); // copy jpeg to payload

      JPEG_EncodeOutputResume();

      /* Set MARKER bit in RTP header on the last packet of an image */
      rtphdr->pt = RTP_PAYLOAD_TYPE| ((RTP_PAYLOAD_SIZE_MAX != val) ? RTP_MARKER_BIT : 0);
      
      while (1)
      /* Send RTP stream packet */
      if (sendto(sock_id, RTP_struct.rtp_send_packet, sizeof(struct rtp_hdr_t) + rtp_payload_size + 8, 0, (struct sockaddr *)net_dest, sizeof(struct sockaddr)) > 0)
      {
        /* Increment the sequence number */
        rtphdr->seq = htons(ntohs(rtphdr->seq) + 1);

        /* Increment the offset in the RTP/JPEG header by the offset of the current packet */
        RTP_struct.Offset += rtp_payload_size;
        break;
      }

      if (rtphdr->pt == 0x9A) {
        xTaskNotifyGive(simDCMItaskHandle);
      }
    }
  }
}

/**
  * @brief  close RTP connection 
  * @param  None
  * @retval None
  */
uint32_t RTP_Close_Connection(void)
{
  uint32_t res_status = CONNECT_ERROR;
  
  res_status = closesocket(RTP_struct.sock_id);
  
  if(res_status == 0)
  {
    /* RTP connection is closed */
    res_status = CONNECT_CLOSED;
  }
  
  /* Close one end of a full-duplex connection */
  shutdown(RTP_struct.sock_id, 0);
  
  return res_status;
}

/**
  * @brief  Close RTP connection, Stop and reset Camera
  * @param  none
  * @retval 0 sucess, 1 error
  */                          
uint32_t RTP_Stop(void)
{
 uint32_t res_status = 1;
 
 /* close the RTP connection */
 if(RTP_Close_Connection() == CONNECT_CLOSED)
 {
   /* Stop the camera */
   //BSP_CAMERA_Stop();
   
   /* Hardware Reset of the camera */
   //BSP_CAMERA_HwReset();

   osThreadTerminate(simDCMItaskHandle);
   
   res_status = 0;
 }
 
 return res_status;
}

/**
  * @brief  Return the RTP state
  * @param  RTP_struct_ptr: pointer to a RTP_HandleTypeDef structure that contains
  *               the information for the RTP.
  * @retval RTP state
  */
RTP_StatusTypeDef RTP_GetState(RTP_HandleTypeDef *RTP_struct_ptr)
{
  return RTP_struct_ptr->State;
}


/**
* @brief Function implementing the simDCMItask thread.
* @param argument: Not used
* @retval None
*/
void StartSimDCMItask(void const * argument)
{
  printf("StartSimDCMItask\r\n");
  /* USER CODE BEGIN StartSimDCMItask */
  uint32_t round = 0;
  /* Infinite loop */
  for(;;)
  {
    printf("Sim Start\r\n");
    uint32_t *startF = (uint32_t*)buffCAM;
    for (uint32_t i = 0; i < 0xFF000000; i+=(0xFF000000/FRAME_SIZE_HEIGHT)) {
      if (i % ((0xFF000000/FRAME_SIZE_HEIGHT) *8) == 0) {
        startF = (uint32_t*)buffCAM;
      }
      for (uint32_t j = 0; j < 0xFF00; j+=(0xFF00/(FRAME_SIZE_WIDTH*2/4))) {
        *startF++ = round | (j & 0xFF00) | (i & 0xFF000000);
      }
      //printf("Sim Line\r\n");
      HAL_DCMI_LineEventCallback(&hdcmi);
      //HAL_Delay(1000/FRAME_SIZE_HEIGHT);
      //osDelay(5);
    }
    printf("Sim Frame\r\n");
    HAL_DCMI_FrameEventCallback(&hdcmi);
    round+=(4) | (4<<16);
    round&=0xFF| (0xFF<<16);
    while (1) {
      if( ulTaskNotifyTake( pdFALSE, 1000 ) != 0 ) {
        break;
      }
    }
    //osDelay(10);
  }
  /* USER CODE END StartSimDCMItask */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
