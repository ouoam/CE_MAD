/*
 * ov7670.c
 *
 *  Created on: 2017/08/25
 *      Author: take-iwiw
 *
 * mix with https://github.com/espressif/esp32-camera/blob/master/sensors/ov7670.c
 * mix with https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov7670.c
 */
#include "ov7670.h"
#include "ov7670Reg.h"

#define SLAVE_ADDR 0x42

/*** Internal Const Values, Macros ***/
#define OV7670_QVGA_WIDTH  320
#define OV7670_QVGA_HEIGHT 240


/*** Internal Static Variables ***/
static DCMI_HandleTypeDef *sp_hdcmi;
static I2C_HandleTypeDef  *sp_hi2c;
static uint32_t    s_destAddressForContiuousMode;
static void (* s_cbHsync)(uint32_t h);
static void (* s_cbVsync)(uint32_t v);

/*** Internal Function Declarations ***/
static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data);
static uint8_t SCCB_Read(uint8_t regAddr);


static int ov7670_clkrc = 0x01;


/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
struct regval_list {
	uint8_t reg_num;
	uint8_t value;
};

static struct regval_list ov7670_default_regs[] = {
    /* Sensor automatically sets output window when resolution changes. */    
    {TSLB, 0x04}, 
    
  //  /* Frame rate 30 fps at 12 Mhz clock */    
	{CLKRC, 0x02},  // 00->15 01->7.5 02->5 03->3.75 04->3 05->2.5 06->2.14
	{DBLV,  0x3A},  

    {COM10, COM10_VSYNC_NEG | COM10_PCLK_MASK},

    /* Improve white balance */ 
	{COM4, 0x40},  
    
    /* Improve color */   
    {RSVD_B0, 0x84},  

    /* Enable 50/60 Hz auto detection */
    {COM11, COM11_EXP|COM11_HZAUTO}, 

    /* Disable some delays */
	{HSYST, 0},
    {HSYEN, 0},   

    {MVFP, MVFP_SUN}, 

	/* More reserved magic, some of which tweaks white balance */
	{AWBC1, 0x0a},		
    {AWBC2, 0xf0},
	{AWBC3, 0x34},		
    {AWBC4, 0x58},
	{AWBC5, 0x28},		
    {AWBC6, 0x3a},
	
    {AWBCTR3, 0x0a},		
    {AWBCTR2, 0x55},
	{AWBCTR1, 0x11},		
    {AWBCTR0, 0x9e}, 

    {COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN|COM8_AWB_EN},

    /* End marker is FF because in ov7670 the address of GAIN 0 and default value too. */
    {0xFF, 0xFF},  
};

static struct regval_list ov7670_default_regs2[] = {//from the linux driver

	{COM10, 0x0 }, // COM10_VS_NEG

	//{ CLKRC, 0x1 },	/* OV: clock scale (30 fps) */

	{0xff, 0xff},	/* END MARKER */
};


static struct regval_list ov7670_fmt_yuv422[] = {
	{ COM7,     0x0                         },  /* Selects YUV mode */
	{ RGB444,   0                           },  /* No RGB444 please */
	{ COM1,     0                           },  /* CCIR601 */
	{ COM15,    COM15_R00FF                 },
    { MVFP,     MVFP_SUN                    }, 
	{ COM9,     0x6A                        },  /* 128x gain ceiling; 0x8 is reserved bit */
	{ MTX1,     0x80                        },  /* "matrix coefficient 1" */
	{ MTX2,     0x80                        }, 	/* "matrix coefficient 2" */
	{ MTX3,     0                           },  /* vb */
	{ MTX4,     0x22                        }, 	/* "matrix coefficient 4" */
	{ MTX5,     0x5e                        },  /* "matrix coefficient 5" */
	{ MTX6,     0x80                        },  /* "matrix coefficient 6" */
	{ COM13,    COM13_UVSAT                 },
	{ 0xff,     0xff                        },  /* END MARKER */
};

static struct regval_list ov7670_fmt_rgb565[] = {
	{ COM7,     COM7_FMT_RGB565             },	/* Selects RGB mode */
	{ RGB444,   0                           },	/* No RGB444 please */
	{ COM1,     0x0                         },	/* CCIR601 */
	{ COM15,    COM15_RGB565 |COM15_R00FF   },
    { MVFP,     MVFP_SUN                    },   
	{ COM9,     0x6A                        }, 	/* 128x gain ceiling; 0x8 is reserved bit */
	{ MTX1,     0xb3                        }, 	/* "matrix coefficient 1" */
	{ MTX2,     0xb3                        }, 	/* "matrix coefficient 2" */
	{ MTX3,     0                           },	/* vb */
	{ MTX4,     0x3d                        }, 	/* "matrix coefficient 4" */
	{ MTX5,     0xa7                        }, 	/* "matrix coefficient 5" */
	{ MTX6,     0xe4                        }, 	/* "matrix coefficient 6" */
	{ COM13,    COM13_UVSAT                 },
	{ 0xff,     0xff                        },  /* END MARKER */
};


static struct regval_list ov7670_vga[] = {
    { COM3,                 0x00 },
    { COM14,                0x00 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF0 },
    { SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

static struct regval_list ov7670_qvga[] = {
    { COM3,                 0x04 },
    { COM14,                0x19 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF1 },
    { SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

static struct regval_list ov7670_qqvga[] = {
	{ COM3,                 0x04 }, //DCW enable	
	{ COM14,                0x1a }, //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register	
	{ SCALING_XSC,          0x3a },	
	{ SCALING_YSC,          0x35 },
	{ SCALING_DCWCTR,       0x22 }, //downsample by 4	
	{ SCALING_PCLK_DIV,     0xf2 }, //pixel clock divided by 4	
	{ SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7670_write_array(struct regval_list *vals)
{
int ret = 0;
	
	while ( (vals->reg_num != 0xff || vals->value != 0xff) && (ret == 0) ) {
    ret = SCCB_Write(vals->reg_num, vals->value);
		vals++;
	}

    return ret;
}


/*
 * Calculate the frame control registers.
 */
static int ov7670_frame_control(int hstart, int hstop, int vstart, int vstop)
{
struct regval_list frame[7];

    frame[0].reg_num = HSTART;
    frame[0].value = (hstart >> 3);

    frame[1].reg_num = HSTOP;
    frame[1].value = (hstop >> 3);

    frame[2].reg_num = HREF;
    frame[2].value = (((hstop & 0x07) << 3) | (hstart & 0x07));
    
    frame[3].reg_num = VSTART;
    frame[3].value = (vstart >> 2);
    
    frame[4].reg_num = VSTOP;
    frame[4].value = (vstop >> 2);

    frame[5].reg_num = VREF;
    frame[5].value = (((vstop & 0x02) << 2) | (vstart & 0x02));

    /* End mark */
    frame[5].reg_num = 0xFF;
    frame[5].value = 0xFF;

    return ov7670_write_array(frame);
}

static int reset()
{
    int ret;

    // Reset all registers
    SCCB_Write(COM7, COM7_RESET);

    // Delay 10 ms
    // vTaskDelay(10 / portTICK_PERIOD_MS);
		HAL_Delay(10);

		ret = ov7670_write_array(ov7670_default_regs);
    ret = ov7670_write_array(ov7670_default_regs2);
		//ret = ov7670_write_array(ov7670_default_regs3);

    // Delay
    // vTaskDelay(30 / portTICK_PERIOD_MS);
		HAL_Delay(30);

    return ret;
}

static int set_pixformat(pixformat_t pixformat)
{
int ret;

    switch (pixformat) {
        case PIXFORMAT_RGB565:
        case PIXFORMAT_RGB888:
            ret = ov7670_write_array(ov7670_fmt_rgb565);
        break;
 
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
	    default:
            ret = ov7670_write_array(ov7670_fmt_yuv422);
        break;
    }

    // vTaskDelay(30 / portTICK_PERIOD_MS);
		HAL_Delay(30);

    /*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
    if (pixformat == PIXFORMAT_RGB565) {
        ret = SCCB_Write(CLKRC, ov7670_clkrc); 
    }

    return ret;
}

static int set_framesize(framesize_t framesize)
{
   int ret;

    // store clkrc before changing window settings...
    ov7670_clkrc =  SCCB_Read(CLKRC);
     
	switch (framesize){
        case FRAMESIZE_VGA:
            if( (ret = ov7670_write_array(ov7670_vga)) == 0 ) {
                /* These values from Omnivision */
                ret = ov7670_frame_control(158, 14, 10, 490);
            }
        break;
	    case FRAMESIZE_QVGA:
            if( (ret = ov7670_write_array(ov7670_qvga)) == 0 ) {
                /* These values from Omnivision */
                ret = ov7670_frame_control(158, 14, 10, 490);
            }
        break;
	    case FRAMESIZE_QQVGA:
            if( (ret = ov7670_write_array(ov7670_qqvga)) == 0 ) {
                /* These values from Omnivision */
                ret = ov7670_frame_control(158, 14, 10, 490);
            }
        break; 

        default:
            ret = -1;   
    }

    // vTaskDelay(30 / portTICK_PERIOD_MS);
		HAL_Delay(30);
		

    if (ret == 0) {
        // sensor->status.framesize = framesize;
    }

	return ret;
}

static int set_colorbar(int enable)
{
    uint8_t ret = 0;
    // Read register scaling_xsc
    uint8_t reg = SCCB_Read(SCALING_XSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_XSC_CBAR(reg);

    // Write pattern to SCALING_XSC
    ret = SCCB_Write(SCALING_XSC, reg);

    // Read register scaling_ysc
    reg = SCCB_Read(SCALING_YSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_YSC_CBAR(reg, enable);

    // Write pattern to SCALING_YSC
    ret = ret | SCCB_Write(SCALING_YSC, reg);

    // return 0 or 0xFF
    return ret;
}

static int set_whitebal(int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(COM8);

    // Set white bal on/off
    reg = COM8_SET_AWB(reg, enable);

    // Write back register COM8
    return SCCB_Write(COM8, reg);
}

static int set_gain_ctrl(int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(COM8);

    // Set white bal on/off
    reg = COM8_SET_AGC(reg, enable);

    // Write back register COM8
    return SCCB_Write(COM8, reg);
}

static int set_exposure_ctrl(int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(COM8);

    // Set white bal on/off
    reg = COM8_SET_AEC(reg, enable);

    // Write back register COM8
    return SCCB_Write(COM8, reg);
}

static int set_hmirror(int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(MVFP);

    // Set mirror on/off
    reg = MVFP_SET_MIRROR(reg, enable);

    // Write back register MVFP
    return SCCB_Write(MVFP, reg);
}

static int set_vflip(int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(MVFP);

    // Set mirror on/off
    reg = MVFP_SET_FLIP(reg, enable);

    // Write back register MVFP
    return SCCB_Write(MVFP, reg);
}

/*** External Function Defines ***/
HAL_StatusTypeDef ov7670_init(DCMI_HandleTypeDef *p_hdcmi, I2C_HandleTypeDef *p_hi2c)
{
  sp_hdcmi     = p_hdcmi;
  sp_hi2c      = p_hi2c;
  s_destAddressForContiuousMode = 0;
	
	HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(CAM_PWDN_GPIO_Port, CAM_PWDN_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(CAM_RESET_GPIO_Port, CAM_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  reset();

  printf("[OV7670] pid = %02X\r\n", SCCB_Read(REG_PID));
	printf("[OV7670] ver = %02X\r\n", SCCB_Read(REG_VER));
	
	set_framesize(FRAMESIZE_QQVGA);
	set_pixformat(PIXFORMAT_YUV422);

	/*
	// Retrieve sensor's signature
    sensor->id.MIDH = SCCB_Read(sensor->slv_addr, REG_MIDH);
    sensor->id.MIDL = SCCB_Read(sensor->slv_addr, REG_MIDL);
    sensor->id.PID = SCCB_Read(sensor->slv_addr, REG_PID);
    sensor->id.VER = SCCB_Read(sensor->slv_addr, REG_VER);
		*/

  return HAL_OK;
}

HAL_StatusTypeDef ov7670_startCap(uint32_t capMode, uint32_t destAddress)
{
  ov7670_stopCap();
  if (capMode == OV7670_CAP_CONTINUOUS) {
    /* note: continuous mode automatically invokes DCMI, but DMA needs to be invoked manually */
    s_destAddressForContiuousMode = destAddress;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_CONTINUOUS, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  } else if (capMode == OV7670_CAP_SINGLE_FRAME) {
    s_destAddressForContiuousMode = 0;
    HAL_DCMI_Start_DMA(sp_hdcmi, DCMI_MODE_SNAPSHOT, destAddress, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
  }

  return HAL_OK;
}

HAL_StatusTypeDef ov7670_stopCap()
{
  HAL_DCMI_Stop(sp_hdcmi);
//  HAL_Delay(30);
  return HAL_OK;
}

void ov7670_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v))
{
  s_cbHsync = cbHsync;
  s_cbVsync = cbVsync;
}

//void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
//{
//  printf("FRAME %d\n", HAL_GetTick());
//  if(s_cbVsync)s_cbVsync(s_currentV);
//  if(s_destAddressForContiuousMode != 0) {
//    HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//  }
//  s_currentV++;
//  s_currentH = 0;
//}

//void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
//{
//  printf("VSYNC %d\n", HAL_GetTick());
//  HAL_DMA_Start_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, s_destAddressForContiuousMode, OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2);
//}

//void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
//{
////  printf("HSYNC %d\n", HAL_GetTick());
//  if(s_cbHsync)s_cbHsync(s_currentH);
//  s_currentH++;
//}

/*** Internal Function Defines ***/
static HAL_StatusTypeDef SCCB_Write(uint8_t regAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  do {
    ret = HAL_I2C_Mem_Write(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
  } while (ret != HAL_OK && 0);
  return ret;
}

static uint8_t SCCB_Read(uint8_t regAddr)
{
  HAL_StatusTypeDef ret;
	uint8_t data[2];
  do {
    // HAL_I2C_Mem_Read doesn't work (because of SCCB protocol(doesn't have ack))? */
//    ret = HAL_I2C_Mem_Read(sp_hi2c, SLAVE_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
    ret = HAL_I2C_Master_Transmit(sp_hi2c, SLAVE_ADDR, &regAddr, 1, 100);
    ret |= HAL_I2C_Master_Receive(sp_hi2c, SLAVE_ADDR, data, 1, 100);
  } while (ret != HAL_OK && 0);
  return data[0];
}
