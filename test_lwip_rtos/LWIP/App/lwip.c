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
#include <string.h>

#include "dcmi.h"

#include "lwip/apps/httpd.h"

#include "encode_dma.h"

#include "websocket_server.h"
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
osThreadId simDCMItaskHandle;
osThreadId wsPicTaskHandle;

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim14;

extern uint8_t buffCAM[MAX_INPUT_LINES * FRAME_SIZE_WIDTH * 2];
extern DCMI_HandleTypeDef hdcmi;
extern uint8_t JPEG_buffer[JPEG_BUFFER_SIZE];

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
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

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
        //*startF++ = round | (j & 0xFF00) | (i & 0xFF000000);
        //*startF++ = round;
        *startF++ = (j & 0xFF00) | (i & 0xFF000000);
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
    while (1) {
      if( ulTaskNotifyTake( pdFALSE, 1000 ) != 0 ) {
        break;
      }
    }
//    osDelay(5000);
//    HAL_Delay(5000);
  }
  /* USER CODE END StartSimDCMItask */
}

uint8_t JPEG_buffer2[JPEG_BUFFER_SIZE];

void wsPicTask(void const * argument)
{
  uint32_t len = 0;
  for (;;) {
    if( xTaskNotifyWait( 0, 0, &len, 1000 ) != 0 )
    {
      int ret = ws_server_send_bin_all((char*)JPEG_buffer, len);
      if (ret == 0) break;
      JPEG_EncodeOutputResume();
      if (len != JPEG_BUFFER_SIZE) {
        ws_server_send_text_all("E", 1);
        xTaskNotifyGive(simDCMItaskHandle);
      }
    }
  }
  osThreadTerminate(NULL);
}

// handles websocket events
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char* msg,uint64_t len) {
  //const static char* TAG = "websocket_callback";

  switch(type) {
    case WEBSOCKET_CONNECT:
      {
        osThreadDef(simDCMI, StartSimDCMItask, osPriorityLow, 0, configMINIMAL_STACK_SIZE * 4);
        simDCMItaskHandle = osThreadCreate(osThread(simDCMI), NULL);

        osThreadDef(wsPic, wsPicTask, osPriorityHigh, 0, configMINIMAL_STACK_SIZE * 4);
        wsPicTaskHandle = osThreadCreate(osThread(wsPic), NULL);
      }
      break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:
      //ESP_LOGI(TAG,"client %i sent a disconnect message",num);
      //led_duty(0);
      //break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
      //ESP_LOGI(TAG,"client %i was disconnected",num);
      //break;
    case WEBSOCKET_DISCONNECT_ERROR:
      //ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
      //led_duty(0);
      osThreadTerminate(simDCMItaskHandle);
      osThreadTerminate(wsPicTaskHandle);
      break;
    case WEBSOCKET_TEXT:
      if(len) { // if the message length was greater than zero
        if(len == 1) {
          static int8_t ch11=9;
          static int8_t ch14=9;

          switch(msg[0]) {
            case 'w': ch11++; break;
            case 's': ch11--; break;
            case 'a': ch14++; break;
            case 'd': ch14--; break;
          }

          if (ch11 < 10) ch11 = 10;
          if (ch11 > 30) ch11 = 30;
          if (ch14 < 7) ch14 = 7;
          if (ch14 > 40) ch14 = 40;

          __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, map(0,180,50,250,ch11*10));
          __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, map(0,180,50,250,ch14*10));

          char response[64];
          int len = snprintf(response, sizeof (response),
                  "%d,%d", ch11, ch14);
          if (len < sizeof (response))
            ws_server_send_text_all_from_callback(response, len);
        }
      }
      break;
    case WEBSOCKET_BIN:
      //ESP_LOGI(TAG,"client %i sent binary message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PING:
      //ESP_LOGI(TAG,"client %i pinged us with message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PONG:
      //ESP_LOGI(TAG,"client %i responded to the ping",num);
      break;
  }
}

// serves any clients
static void http_serve(struct netconn *conn) {
  //const static char* TAG = "http_server";
  struct netbuf* inbuf;
  static char* buf;
  static uint16_t buflen;
  static err_t err;

  //netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
  //ESP_LOGI(TAG,"reading from client...");
  err = netconn_recv(conn, &inbuf);
  //ESP_LOGI(TAG,"read from client");
  if(err==ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    if(buf) {

      // default page websocket
      if(strstr(buf,"GET /")
           && strstr(buf,"Upgrade: websocket")) {
        //ESP_LOGI(TAG,"Requesting websocket on /");
        ws_server_add_client(conn,buf,buflen,"/",websocket_callback);
        netbuf_delete(inbuf);
      }
      else {
        //ESP_LOGI(TAG,"Unknown request");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }
    }
    else {
      //ESP_LOGI(TAG,"Unknown request (empty?...)");
      netconn_close(conn);
      netconn_delete(conn);
      netbuf_delete(inbuf);
    }
  }
  else { // if err==ERR_OK
    //ESP_LOGI(TAG,"error on read, closing connection");
    netconn_close(conn);
    netconn_delete(conn);
    netbuf_delete(inbuf);
  }
}

// handles clients when they first connect. passes to a queue
static void server_task(void const * pvParameters) {
  //const static char* TAG = "server_task";
  struct netconn *conn, *newconn;
  static err_t err;

  conn = netconn_new(NETCONN_TCP);
  netconn_bind(conn,NULL,8080);
  netconn_listen(conn);
  //ESP_LOGI(TAG,"server listening");
  do {
    err = netconn_accept(conn, &newconn);
    //ESP_LOGI(TAG,"new client");
    if(err == ERR_OK) {
      if(!newconn) continue;
          http_serve(newconn);
      //http_serve(newconn);
    }
  } while(err == ERR_OK);
  netconn_close(conn);
  netconn_delete(conn);
  //ESP_LOGE(TAG,"task ending, rebooting board");
  //esp_restart();
}

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 88;
  IP_ADDRESS[3] = 99;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 192;
  GATEWAY_ADDRESS[1] = 168;
  GATEWAY_ADDRESS[2] = 88;
  GATEWAY_ADDRESS[3] = 99;

  /* Initilialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

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
  osThreadCreate(osThread(LinkThr), &link_arg);
/* USER CODE END OS_THREAD_DEF_CREATE_CMSIS_RTOS_V1 */

/* USER CODE BEGIN 3 */
  ws_server_start();

  osThreadDef(server_task, server_task, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE * 4);
  osThreadCreate(osThread(server_task), NULL);

  httpd_init();
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
