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


typedef enum {
    PIXFORMAT_RGB565,    // 2BPP/RGB565
    PIXFORMAT_YUV422,    // 2BPP/YUV422
    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
    PIXFORMAT_RGB888,    // 3BPP/RGB888
    PIXFORMAT_RAW,       // RAW
    PIXFORMAT_RGB444,    // 3BP2P/RGB444
    PIXFORMAT_RGB555,    // 3BP2P/RGB555
} pixformat_t;

typedef enum {
    FRAMESIZE_96X96,    // 96x96
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240X240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
    // 3MP Sensors
    FRAMESIZE_FHD,      // 1920x1080
    FRAMESIZE_P_HD,     //  720x1280
    FRAMESIZE_P_3MP,    //  864x1536
    FRAMESIZE_QXGA,     // 2048x1536
    // 5MP Sensors
    FRAMESIZE_QHD,      // 2560x1440
    FRAMESIZE_WQXGA,    // 2560x1600
    FRAMESIZE_P_FHD,    // 1080x1920
    FRAMESIZE_QSXGA,    // 2560x1920
    FRAMESIZE_INVALID
} framesize_t;

HAL_StatusTypeDef ov7670_init(DCMI_HandleTypeDef *p_hdcmi, I2C_HandleTypeDef *p_hi2c);
HAL_StatusTypeDef ov7670_startCap(uint32_t capMode, uint32_t destAddress);
HAL_StatusTypeDef ov7670_stopCap();
void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));

#endif /* OV7670_OV7670_H_ */
