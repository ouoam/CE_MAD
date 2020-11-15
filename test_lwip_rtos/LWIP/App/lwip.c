/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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

/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"

/* USER CODE BEGIN 0 */
#include "lwip/apps/httpd.h"

#include "encode_dma.h"
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
osThreadId simDCMItaskHandle;
osThreadId webSocketTaskHandle;
osThreadId wsPicTaskHandle;

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim14;

extern uint8_t buffCAM[MAX_INPUT_LINES * FRAME_SIZE_WIDTH * 2];
extern DCMI_HandleTypeDef hdcmi;
extern uint8_t JPEG_buffer[JPEG_BUFFER_SIZE];

static QueueHandle_t client_queue;
const static int client_queue_size = 10;
/* USER CODE END 1 */
/* Semaphore to signal Ethernet Link state update */
osSemaphoreId Netif_LinkSemaphore = NULL;
/* Ethernet link thread Argument */
struct link_str link_arg;

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

/* USER CODE BEGIN 2 */

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
    //printf("Sim Start\r\n");
    uint32_t *startF = (uint32_t*)buffCAM;
    for (uint32_t i = 0; i < 0xFF000000; i+=(0xFF000000/FRAME_SIZE_HEIGHT)) {
      if (i % ((0xFF000000/FRAME_SIZE_HEIGHT) *8) == 0) {
        startF = (uint32_t*)buffCAM;
      }
      for (uint32_t j = 0; j < 0xFF00; j+=(0xFF00/(FRAME_SIZE_WIDTH*2/4))) {
        *startF++ = round | (j & 0xFF00) | (i & 0xFF000000);
        //*startF++ = 0x007F007F00 | (j & 0xFF00) | (i & 0xFF000000);
      }
      //printf("Sim Line\r\n");
      HAL_DCMI_LineEventCallback(&hdcmi);
      //HAL_Delay(1000/FRAME_SIZE_HEIGHT);
//      HAL_Delay(500);
    }
    //printf("Sim Frame\r\n");
    HAL_DCMI_FrameEventCallback(&hdcmi);
    round+=(4) | (4<<16);
    round&=0xFF| (0xFF<<16);
//    while (1) {
//      if( ulTaskNotifyTake( pdFALSE, 1000 ) != 0 ) {
//        break;
//      }
//    }
    osDelay(5000);
//    HAL_Delay(5000);
  }
  /* USER CODE END StartSimDCMItask */
}

uint8_t JPEG_buffer2[JPEG_BUFFER_SIZE];

//void wsPicTask(void const * argument)
//{
//  uint32_t len = 0;
//  for (;;) {
//    if( xTaskNotifyWait( 0, 0, &len, 1000 ) != 0 )
//    {
//      //ws_server_send_bin_all((char*)JPEG_buffer, len);
//      websocket_write(pcb, data, data_len, WS_BIN_MODE);
//      JPEG_EncodeOutputResume();
//      if (len != JPEG_BUFFER_SIZE) {
//        osDelay(100000);
//        //ws_server_send_text_all("E", 1);
//        websocket_write(pcb, "E", 1, WS_TEXT_MODE);
//        xTaskNotifyGive(simDCMItaskHandle);
//      }
//
//    }
//  }
//}

void websocket_task(void const * argument)
{
    struct tcp_pcb *pcb = (struct tcp_pcb *) argument;

    static int c = 523;

    for (;;) {
        if (pcb == NULL || pcb->state != ESTABLISHED) {
            printf("Connection closed, deleting task\n");
//            osThreadTerminate(simDCMItaskHandle);
            break;
        }

        int uptime = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        int heap = (int) xPortGetFreeHeapSize();

        /* Generate response in JSON format */
        char response[64];
        int len = snprintf(response, sizeof (response),
                "{\"uptime\" : \"%d\","
                " \"heap\" : \"%d\","
                " \"led\" : \"%d\"}", uptime, heap, c);
        c++;
        if (len < sizeof (response))
            websocket_write(pcb, (unsigned char *) response, len, WS_TEXT_MODE);

        osDelay(2000 / portTICK_PERIOD_MS);
    }

    osThreadTerminate(NULL);
}

/**
 * This function is called when websocket frame is received.
 *
 * Note: this function is executed on TCP thread and should return as soon
 * as possible.
 */
void websocket_cb(struct tcp_pcb *pcb, uint8_t *data, u16_t data_len, uint8_t mode)
{
    printf("[websocket_callback]:\n%.*s\n", (int) data_len, (char*) data);

    websocket_write(pcb, data, data_len, WS_BIN_MODE);
}

/**
 * This function is called when new websocket is open and
 * creates a new websocket_task if requested URI equals '/stream'.
 */
void websocket_open_cb(struct tcp_pcb *pcb, const char *uri)
{
    printf("WS URI: %s\n", uri);

    osThreadDef(webSocket, websocket_task, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 4);
    webSocketTaskHandle = osThreadCreate(osThread(webSocket), (void *)pcb);
}
/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* Initilialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization with DHCP (IPv4) */
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);

  /* create a binary semaphore used for informing ethernetif of frame reception */
  osSemaphoreDef(Netif_SEM);
  Netif_LinkSemaphore = osSemaphoreCreate(osSemaphore(Netif_SEM) , 1 );

  link_arg.netif = &gnetif;
  link_arg.semaphore = Netif_LinkSemaphore;
  /* Create the Ethernet link handler thread */
/* USER CODE BEGIN OS_THREAD_DEF_CREATE_CMSIS_RTOS_V1 */
  osThreadDef(LinkThr, ethernetif_set_link, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  osThreadCreate (osThread(LinkThr), &link_arg);
/* USER CODE END OS_THREAD_DEF_CREATE_CMSIS_RTOS_V1 */

  /* Start DHCP negotiation for a network interface (IPv4) */
  dhcp_start(&gnetif);

/* USER CODE BEGIN 3 */
  // RTSP_Init();

  websocket_register_callbacks((tWsOpenHandler) websocket_open_cb, (tWsHandler) websocket_cb);
  httpd_init();

  osThreadDef(simDCMI, StartSimDCMItask, osPriorityLow, 0, configMINIMAL_STACK_SIZE * 4);
  simDCMItaskHandle = osThreadCreate(osThread(simDCMI), NULL);

//  osThreadDef(wsPic, wsPicTask, osPriorityHigh, 0, configMINIMAL_STACK_SIZE * 4);
//  wsPicTaskHandle = osThreadCreate(osThread(wsPic), NULL);
/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
