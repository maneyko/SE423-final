#include <xdc/std.h>

#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include <math.h> 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines


#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"
#include "pru.h"
#include "PRUcode\rgb_final_bin.h"
#include "mcbsp_com.h"
#include "ColorVision.h"
#include "sharedmem.h"
#include "ladar.h"


#include "projectinclude.h"

int updateLCD = 1;
uint8_t testvar = 0;

int *pic_data;

bgr *Image_data;
bgr *Linux_Image;
bgr *LCD_Image;

unsigned char i2cdata[40];


// ===== BEGIN Student Variables =====

int r = 0;
int c = 0;
bgr pixel;
int rowsum = 0;
int colsum = 0;

int centroid_x = 0;
int centroid_y = 0;
int nwhite = 0;

// These are sent to `user_lab8_dgnava2_maneyko2.c`
int Cx = 0;
int Cy = 0;
int Nw = 0;

int prnt_flag = 0;

// ===== END Student Variables =====


unsigned char *Thres_Image;

sharedmemstruct *ptrshrdmem;

extern EDMA3_CCRL_Regs *EDMA3_1_Regs;

void init_ColorVision(void) {

    // initialize ov6620 cmos camera
    i2cdata[0] = 0x01;
    i2cdata[1] = 0x8F;  // Blue Gain control (default 0x80)
    i2cdata[2] = 0x8F;  // Red Gain Control
    i2cdata[3] = 0x80;  // Saturation
    i2cdata[4] = 0x00;  // Reserved
    i2cdata[5] = 0x4f;  // Contrast
    i2cdata[6] = 0x9f; // Brightness
    i2cdata[7] = 0xCF; // Sharpness  (default 0xC6)
    i2cdata[8] = 0x00; // Reserved
    i2cdata[9] = 0x00; // Reserved
    i2cdata[10] = 0x00; // Reserved
    i2cdata[11] = 0x00; // Reserved
    i2cdata[12] = 0x20; // AWB - Blue
    i2cdata[13] = 0x20; // AWB - Red
    i2cdata[14] = 0x0D; // COMR
    i2cdata[15] = 0x05; // COMS
    i2cdata[16] = 0x9A; // AEC
    i2cdata[17] = 0x01; // CLKRC
    i2cdata[18] = 0x28; // COMA
    i2cdata[19] = 0x01; // 0x01; // COMB
    I2C_write(I2C0, 0x60, i2cdata, 20, SET_STOP_BIT_AFTER_WRITE);

    i2cdata[0] = 0x20;
    i2cdata[1] = 0x01; // COME
    I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);

    i2cdata[0] = 0x28;
    i2cdata[1] = 0x81; // COMH
    I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);

    i2cdata[0] = 0x39;
    // changed to PCLK always on.
    i2cdata[1] = 0x00; //0x40; // COML
    I2C_write(I2C0, 0x60, i2cdata, 2, SET_STOP_BIT_AFTER_WRITE);


    VPIF_initReceive(VIDEO_CONN_CAMERA);

    // PRU setup
    EVMOMAPL138_lpscTransition(PSC0, DOMAIN0, LPSC_DMAX, PSC_ENABLE);
    PRU_stop(PRU0);
    PRU_reset(PRU0);
    PRU_load(PRU0_PROG, PRUCode, sizeof(PRUCode)/sizeof(uint32_t));
    PRU_reset(PRU0);
    PRU_run(PRU0);

    // VPIF (95) interrupt serviced by INT4
    ICR = 0x010; // clear pending interrupts
    IER |= 0x010; // enable interrupt on line

    // PRU (6) interrupt serviced by INT6
    ICR = 0x040; // clear pending interrupts
    IER |= 0x040; // enable interrupt on line

    pic_data = (int *)ADDR_VIDEO_DATA_BASE;

    Image_data = (bgr *)(IMAGE_DATA_MEM);
    
    Thres_Image = (unsigned char *)(THRES_IMAGE_MEM);

    Linux_Image = (bgr *)(ADDR_VIDEO_DATA_BASE+LINUX_IMAGE_OFFSET);

    LCD_Image = (bgr *)(ADDR_VIDEO_DATA_BASE+LCD_IMAGE_OFFSET);

    ptrshrdmem = (sharedmemstruct *)SHARED_MEM;


    while (CHKBIT(VPIF->INTSTAT,INT_FRAME_CH1) == 0) {}
    SETBIT(VPIF->INTSTATCLR, INT_FRAME_CH1);

}

int VPIF_beginning_line = 1;
void VPIF_HWI(void) {

    if (VPIF_beginning_line == 1) {
        // Invalidate Destination
        Cache_inv((void *)Image_data,IMAGE_ROWS*IMAGE_COLUMNS*3, Cache_Type_ALL, EDMA3_CACHE_WAIT);
        //BCACHE_inv((void *)Image_data,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
        VPIF_beginning_line = 0;
    }
    PRU_wakeup(PRU0);
    // at times I had to run the below lines to wake up the PRU ????
//  PRU_stop(PRU0);
//  PRU_run(PRU0);
    SETBIT(VPIF->INTSTATCLR, INT_FRAME_CH1);
}

void PRU_HWI(void) {
    VPIF_beginning_line = 1;
    Swi_post(SWI_vision);
    EVTCLR0 |= PRU_INTCLR;

}

// This function is called each time the DSP receives a new picture 
void userProcessColorImageFunc_laser(bgr *ptrImage) {


    if (ptrImage != NULL) {

        rowsum = 0;
        colsum = 0;
        nwhite = 0;

        // add your vision processing code here

        for (r = 0; r < (IMAGE_ROWS-50); r++) {
            for (c = 0; c < IMAGE_COLUMNS; c++) {
                pixel = ptrImage[r*IMAGE_COLUMNS + c];
                if ((pixel.red > 200) && (pixel.green > 200) && (pixel.blue > 200)) {  // Check if pixel is white
                    nwhite++;
                    rowsum += r;
                    colsum += c;
//                    ptrImage[r*IMAGE_COLUMNS + c].blue = 0;
//                    ptrImage[r*IMAGE_COLUMNS + c].green = 0;
//                    ptrImage[r*IMAGE_COLUMNS + c].red = 255;

                }
                else {
//                    ptrImage[r*IMAGE_COLUMNS + c].red = 0;
//                    ptrImage[r*IMAGE_COLUMNS + c].green = 0;
//                    ptrImage[r*IMAGE_COLUMNS + c].blue = 0;
                }
            }
        }

        if (nwhite == 0) {
            centroid_x = IMAGE_COLUMNS/2;
            centroid_y = IMAGE_ROWS/2;
        }
        else {
            centroid_x = colsum / nwhite;
            centroid_y = rowsum / nwhite;
        }

        for (r = centroid_y-1; r < centroid_y+1; r++) {
            ptrImage[r*IMAGE_COLUMNS + centroid_x].red = 0;
            ptrImage[r*IMAGE_COLUMNS + centroid_x].green = 255;
            ptrImage[r*IMAGE_COLUMNS + centroid_x].blue = 0;
        }

        for (c = centroid_x-1; c < centroid_x+1; c++) {
            ptrImage[centroid_y*IMAGE_COLUMNS + c].red = 0;
            ptrImage[centroid_y*IMAGE_COLUMNS + c].green = 255;
            ptrImage[centroid_y*IMAGE_COLUMNS + c].blue = 0;
        }

        if (prnt_flag == 0) {
            // Make
//            Cx = -(centroid_x-(IMAGE_COLUMNS/2));
            Cx = IMAGE_COLUMNS/2 - centroid_x;
            Cy = centroid_y - IMAGE_ROWS/2;
            Nw = nwhite;
            prnt_flag = 1;
        }

        // Send image to Color LCD if LCD ready for new data
        if (updateLCD) {

            updateLCD = 0;
            Cache_inv((void *)(ADDR_VIDEO_DATA_BASE+0x1A900),IMAGE_ROWS*IMAGE_COLUMNS*3, Cache_Type_ALL, EDMA3_CACHE_WAIT);
            //BCACHE_inv((void *)(ADDR_VIDEO_DATA_BASE+0x1A900),IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
            // Flush or write back source
            Cache_wb((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3, Cache_Type_ALL, EDMA3_CACHE_WAIT);
            //BCACHE_wb ((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
            //Need to clean the cache here
            EDMA3_1_Regs->PARAMENTRY[33].OPT = 0x0011E00C;
            EDMA3_1_Regs->PARAMENTRY[33].SRC = (unsigned int)Image_data;
            EDMA3_1_Regs->PARAMENTRY[33].A_B_CNT = 0x004004A4;   //Maybe to ACNT = 1188 and BCNT = 64
            EDMA3_1_Regs->PARAMENTRY[33].DST = (ADDR_VIDEO_DATA_BASE+LCD_IMAGE_OFFSET);
            EDMA3_1_Regs->PARAMENTRY[33].SRC_DST_BIDX = 0x04A404A4;
            EDMA3_1_Regs->PARAMENTRY[33].LINK_BCNTRLD = 0x0000FFFF;  // Null link
            EDMA3_1_Regs->PARAMENTRY[33].SRC_DST_CIDX = 0x0;
            EDMA3_1_Regs->PARAMENTRY[33].CCNT = 0x1;  //Last command triggers transmission

        }

        // If Linux is ready for another full 176X144 RGB image start the EDMA transfer of the image to external memory
        if (GET_IMAGE_TO_LINUX) {

            // Invalidate Destination
            Cache_inv((void *)Linux_Image,IMAGE_ROWS*IMAGE_COLUMNS*3, Cache_Type_ALL, EDMA3_CACHE_WAIT);
            //BCACHE_inv((void *)Linux_Image,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);
            // Flush or write back source
            Cache_wb((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3, Cache_Type_ALL, EDMA3_CACHE_WAIT);
            //BCACHE_wb ((void *)ptrImage,IMAGE_ROWS*IMAGE_COLUMNS*3,EDMA3_CACHE_WAIT);

            EDMA3_1_Regs->PARAMENTRY[32].OPT = 0x0011F00C;
            EDMA3_1_Regs->PARAMENTRY[32].SRC = (unsigned int)Image_data;
            EDMA3_1_Regs->PARAMENTRY[32].A_B_CNT = 0x004004A4;   //Maybe to ACNT = 1188 and BCNT = 64
            EDMA3_1_Regs->PARAMENTRY[32].DST = (ADDR_VIDEO_DATA_BASE+LINUX_IMAGE_OFFSET);
            EDMA3_1_Regs->PARAMENTRY[32].SRC_DST_BIDX = 0x04A404A4;
            EDMA3_1_Regs->PARAMENTRY[32].LINK_BCNTRLD = 0x0000FFFF;  // Null link
            EDMA3_1_Regs->PARAMENTRY[32].SRC_DST_CIDX = 0x0;
            EDMA3_1_Regs->PARAMENTRY[32].CCNT = 0x1;  //Last command triggers transmission

        }

    }  // Ends if statement to see if image pointer is null

}

void vision(void) {
    
    //SETLED3;

    // 144X176 BGR image ready for processing
    userProcessColorImageFunc_laser(Image_data);

    //CLRLED3;
}

