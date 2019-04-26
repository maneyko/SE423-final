/*
 *  ======== main.c ========
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines

#include <xdc/std.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>


#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "evmomapl138_spi.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "ladar.h"
#include "xy.h"
#include "MatrixMath.h"
#include "projectinclude.h"

#define FEETINONEMETER 3.28083989501312
extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

volatile uint32_t index;

// test variables
extern float enc1;  // Left motor encoder
extern float enc2;  // Right motor encoder
extern float enc3;
extern float enc4;
extern float adcA0;  // ADC A0 - Gyro_X -400deg/s to 400deg/s  Pitch
extern float adcB0;  // ADC B0 - External ADC Ch4 (no protection circuit)
extern float adcA1;  // ADC A1 - Gyro_4X -100deg/s to 100deg/s  Pitch
extern float adcB1;  // ADC B1 - External ADC Ch1
extern float adcA2;  // ADC A2 -    Gyro_4Z -100deg/s to 100deg/s  Yaw
extern float adcB2;  // ADC B2 - External ADC Ch2
extern float adcA3;  // ADC A3 - Gyro_Z -400deg/s to 400 deg/s  Yaw
extern float adcB3;  // ADC B3 - External ADC Ch3
extern float adcA4;  // ADC A4 - Analog IR1
extern float adcB4;  // ADC B4 - USONIC1
extern float adcA5;  // ADC A5 -    Analog IR2
extern float adcB5;  // ADC B5 - USONIC2
extern float adcA6;  // ADC A6 - Analog IR3
extern float adcA7;  // ADC A7 - Analog IR4
extern float compass;
extern float switchstate;

extern sharedmemstruct *ptrshrdmem;

float vref = 0;
float turn = 0;

int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float LVvalue1 = 0;
float LVvalue2 = 0;
int new_LV_data = 0;

int newnavdata = 0;
float newvref = 0;
float newturn = 0;

extern sharedmemstruct *ptrshrdmem;

float x_pred[3][1] = {{0},{0},{0}};                 // predicted state

//more kalman vars
float B[3][2] = {{1,0},{1,0},{0,1}};            // control input model
float u[2][1] = {{0},{0}};          // control input in terms of velocity and angular velocity
float Bu[3][1] = {{0},{0},{0}}; // matrix multiplication of B and u
float z[3][1];                          // state measurement
float eye3[3][3] = {{1,0,0},{0,1,0},{0,0,1}};   // 3x3 identity matrix
float K[3][3] = {{1,0,0},{0,1,0},{0,0,1}};      // optimal Kalman gain
#define ProcUncert 0.0001
#define CovScalar 10
float Q[3][3] = {{ProcUncert,0,ProcUncert/CovScalar},
                 {0,ProcUncert,ProcUncert/CovScalar},
                 {ProcUncert/CovScalar,ProcUncert/CovScalar,ProcUncert}};   // process noise (covariance of encoders and gyro)
#define MeasUncert 1
float R[3][3] = {{MeasUncert,0,MeasUncert/CovScalar},
                 {0,MeasUncert,MeasUncert/CovScalar},
                 {MeasUncert/CovScalar,MeasUncert/CovScalar,MeasUncert}};   // measurement noise (covariance of LADAR)
float S[3][3] = {{1,0,0},{0,1,0},{0,0,1}};  // innovation covariance
float S_inv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};  // innovation covariance matrix inverse
float P_pred[3][3] = {{1,0,0},{0,1,0},{0,0,1}}; // predicted covariance (measure of uncertainty for current position)
float temp_3x3[3][3];               // intermediate storage
float temp_3x1[3][1];               // intermediate storage
float ytilde[3][1];                 // difference between predictions

// deadreckoning
float vel1 = 0,vel2 = 0;
float vel1old = 0,vel2old = 0;
float enc1old = 0,enc2old = 0;

// SETTLETIME should be an even number and divisible by 3
#define SETTLETIME 6000
int settlegyro = 0;
float gyro_zero = 0;
float gyro_angle = 0;
float old_gyro = 0;
float gyro_drift = 0;
float gyro = 0;
int gyro_degrees = 0;
float gyro_radians = 0.0;
float gyro_x = 0,gyro_y = 0;
float gyro4x_gain = 1;

int statePos = 0;   // index into robotdest
int robotdestSize = 8;  // number of positions to use out of robotdest
pose robotdest[8];  // array of waypoints for the robot

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];        // in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
extern pose ROBOTps;
extern pose LADARps;
extern float newLADARdataX[LADAR_MAX_DATA_SIZE];
extern float newLADARdataY[LADAR_MAX_DATA_SIZE];
float LADARdataX[LADAR_MAX_DATA_SIZE];
float LADARdataY[LADAR_MAX_DATA_SIZE];
extern int newLADARdata;

// Optitrack Variables
int trackableIDerror = 0;
int firstdata = 1;
volatile int new_optitrack = 0;
volatile float previous_frame = -1;
int frame_error = 0;
volatile float Optitrackdata[OPTITRACKDATASIZE];
pose OPTITRACKps;
float temp_theta = 0.0;
float tempOPTITRACK_theta = 0.0;
volatile int temp_trackableID = -1;
int trackableID = -1;
int errorcheck = 1;

// ======================================================== START Student Variables ========================================================

// ====== Start Wall Following and Dead Reckoning Student Variables ========

int lo = 0, hi = 0;

float front_180 = 10000.0;
float front_120 = 10000.0;
float front_90 = 10000.0;
float front_60 = 10000.0;
float front_30 = 10000.0;

float left_30 = 10000.0;
float left_50 = 10000.0;
float left_side = 10000.0;
float left_rear = 10000.0;
float left_forward = 10000.0;

float right_30 = 10000.0;
float right_50 = 10000.0;
float right_side = 10000.0;
float right_rear = 10000.0;
float right_forward = 10000.0;

float obstacle = 240; //set to min distance before obstacle is detected
//float obstacle2 = 800;

float ref_right_wall = 250;
float left_turn_Start_threshold = 275;
float left_turn_Stop_threshold = 250;
float Kp_right_wall = -0.005;
float Kp_front_wall = -0.003;
float turn_command_saturation = 4.0;
float forward_velocity = 1.5;

int pval = 1;  // Initial state
int ppval = 1;
long tc = 0;  // Personal timechecking variable.
int dt = 0;
int dt_wf = 500;

float v1_x = 0.0;
float v1_y = 0.0;
float v1_theta = 0.0;
float Rv1_theta = 0.0;
float v1_mag = 0.0;

float mytheta = 0.0;
float v1_theta2 = 0.0;
float diff_angle = 0.0;

float hit_x = 0.0;
float hit_y = 0.0;
float hit_theta = 0.0;
float hit_mag = 0.0;

float rad2deg(float radval) {
    return (float)(radval * 180.0 / PI);
}

float deg2rad(float degval) {
    return (float)(degval * PI / 180.0);
}

// Function Prototypes (at bottom)
float _min_ladar_t = 10000.0;
int _ii = 0;
float min_LADAR(int lo, int hi) {
    _min_ladar_t = 10000.0;
    for (_ii = lo; _ii <= hi; _ii++)
        _min_ladar_t = MIN(LADARdistance[_ii], _min_ladar_t);
    return _min_ladar_t;
}


float retval = 0.0;
float get_adjustment_angle(void) {
    // Takes global varaibles:
    // v1_x, v1_y, v1_theta, mytheta
    retval = 0.0;
    v1_theta2 = v1_theta;

    if (v1_x >= 0 && v1_y >= 0)
        v1_theta2 = v1_theta2;

    else if ((v1_x < 0 && v1_y >= 0)
            || (v1_x < 0 && v1_y < 0))
        v1_theta2 = v1_theta2 + 180;

    else if (v1_x > 0 && v1_y < 0)
        v1_theta2 = v1_theta2 + 360;

    retval = v1_theta2 - mytheta;

    if (retval > 180)
        retval -= 360.0;

    else if (0 <= retval && retval <= 180)
        Rv1_theta = retval;
    else if (-180 <= retval && retval < 0)
        Rv1_theta = retval;
    else if (retval < -180)
        retval += 360;

    diff_angle = -retval;
    return retval;
}



//float min_LADAR(int lo, int hi);
float LeftRight = 0.0;

// ======================================================== END Student Variables ========================================================

pose UpdateOptitrackStates(pose localROBOTps, int * flag);

void ComWithLinux(void) {

    int i = 0;
    Task_sleep(100);

    while(1) {

        Cache_inv((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
        //BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

        if (GET_DATA_FROM_LINUX) {

            if (newnavdata == 0) {
                newvref = ptrshrdmem->Floats_to_DSP[0];
                newturn = ptrshrdmem->Floats_to_DSP[1];
                newnavdata = 1;
            }

            CLR_DATA_FROM_LINUX;

        }

        if (GET_LVDATA_FROM_LINUX) {

            if (ptrshrdmem->DSPRec_size > 256) ptrshrdmem->DSPRec_size = 256;
            for (i=0;i<ptrshrdmem->DSPRec_size;i++) {
                fromLinuxstring[i] = ptrshrdmem->DSPRec_buf[i];
            }
            fromLinuxstring[i] = '\0';

            if (new_LV_data == 0) {
                sscanf(fromLinuxstring,"%f%f",&LVvalue1,&LVvalue2);
                new_LV_data = 1;
            }

            CLR_LVDATA_FROM_LINUX;

        }

        if ((tskcount%6)==0) {
            if (GET_LVDATA_TO_LINUX) {

                // Default
                //                ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"1.0 1.0 1.0 1.0");
                // you would do something like this
                ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",
                                                   ROBOTps.x, ROBOTps.y, (float)pval, (float)pval);
                //ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",var1,var2,var3,var4);

                for (i=0;i<ptrshrdmem->DSPSend_size;i++) {
                    ptrshrdmem->DSPSend_buf[i] = toLinuxstring[i];
                }

                // Flush or write back source
                Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
                //BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

                CLR_LVDATA_TO_LINUX;

            }
        }

        if (GET_DATAFORFILE_TO_LINUX) {
            if (GET_DATAFORFILE_TO_LINUX) {
                // First make sure all scratch elements are zero
                for (i=0;i<500;i++) {
                    ptrshrdmem->scratch[i] = 0;
                }
                // Write LADARdataX to scratch
                for (i=0;i<228;i++) {
                    ptrshrdmem->scratch[i] = LADARdataX[i];
                }
                // Write LADARdataY to scratch
                for (i=0;i<228;i++) {
                    ptrshrdmem->scratch[228+i] = LADARdataY[i];
                }
                // Flush or write back source
                Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);

                CLR_DATAFORFILE_TO_LINUX;
            }

        }

        tskcount++;
        Task_sleep(40);

    }
}


/*
 *  ======== main ========
 */
Int main()
{ 
    int i = 0;

    // unlock the system config registers.
    SYSCONFIG->KICKR[0] = KICK0R_UNLOCK;
    SYSCONFIG->KICKR[1] = KICK1R_UNLOCK;

    SYSCONFIG1->PUPD_SEL |= 0x10000000;  // change pin group 28 to pullup for GP7[12/13] (LCD switches)

    // Initially set McBSP1 pins as GPIO ins
    CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[1], 0x88888880);  // This is enabling the McBSP1 pins

    CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
    SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13
    CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
    SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13


    //Rick added for LCD DMA flagging test
    GPIO_setDir(GPIO_BANK0, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);

    GPIO_setDir(GPIO_BANK0, GPIO_PIN0, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN1, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN2, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN3, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN4, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN5, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN6, GPIO_INPUT);

    GPIO_setDir(GPIO_BANK7, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN9, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN10, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN11, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN12, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN13, GPIO_INPUT);

    GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN9, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN10, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN11, OUTPUT_HIGH);

    CLRBIT(SYSCONFIG->PINMUX[13], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[13], 0x88888811); //Set GPIO 6.8-13 to GPIOs and IMPORTANT Sets GP6[15] to /RESETOUT used by PHY, GP6[14] CLKOUT appears unconnected

#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

    GPIO_setDir(GPIO_BANK6, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN9, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN10, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN11, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN12, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN13, GPIO_INPUT);


    while ((T1_TGCR & 0x7) != 0x7) {
        for (index=0;index<50000;index++) {}  // small delay before checking again

    }



    USTIMER_init();


    // Turn on McBSP1
    EVMOMAPL138_lpscTransition(PSC1, DOMAIN0, LPSC_MCBSP1, PSC_ENABLE);

    // If Linux has already booted It sets a flag so no need to delay
    if ( GET_ISLINUX_BOOTED == 0) {
        USTIMER_delay(4*DELAY_1_SEC);  // delay allowing Linux to partially boot before continuing with DSP code
    }

    // init the us timer and i2c for all to use.
    I2C_init(I2C0, I2C_CLK_100K);
    init_ColorVision();
    init_LCD_mem(); // added rick

    EVTCLR0 = 0xFFFFFFFF;
    EVTCLR1 = 0xFFFFFFFF;
    EVTCLR2 = 0xFFFFFFFF;
    EVTCLR3 = 0xFFFFFFFF;

    init_DMA();
    init_McBSP();

    init_LADAR();

    CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[1], 0x22222220);  // This is enabling the McBSP1 pins

    CLRBIT(SYSCONFIG->PINMUX[5], 0x00FF0FFF);
    SETBIT(SYSCONFIG->PINMUX[5], 0x00110111);  // This is enabling SPI pins

    CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
    SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13
    CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
    SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13

    init_LCD();

    LADARps.x = 3.5/12; // 3.5/12 for front mounting
    LADARps.y = 0;
    LADARps.theta = 1;  // not inverted

    OPTITRACKps.x = 0;
    OPTITRACKps.y = 0;
    OPTITRACKps.theta = 0;

    for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
    { LADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.

    // ROBOTps will be updated by Optitrack during gyro calibration
    // TODO: specify the starting position of the robot
    ROBOTps.x = 0;          //the estimate in array form (useful for matrix operations)
    ROBOTps.y = 0;
    ROBOTps.theta = 0;  // was -PI: need to flip OT ground plane to fix this
    x_pred[0][0] = ROBOTps.x; //estimate in structure form (useful elsewhere)
    x_pred[1][0] = ROBOTps.y;
    x_pred[2][0] = ROBOTps.theta;

    // Defined destinations that moves the robot around and outside the course
    //    robotdest[0].x = -2;  robotdest[0].y = 6;
    //    robotdest[1].x = -4;  robotdest[1].y = 2;
    //    //middle of bottom
    //    robotdest[2].x = 0;       robotdest[2].y = 2;
    //    //outside the course
    //    robotdest[3].x = 0;       robotdest[3].y = -3;
    //    //back to middle
    //    robotdest[4].x = 0;       robotdest[4].y = 2;
    //    robotdest[5].x = 4;       robotdest[5].y = 2;
    //    robotdest[6].x = 4;       robotdest[6].y = 10;
    //    robotdest[7].x = 0;       robotdest[7].y = 9;


    // Points for comp
    robotdest[0].x =  0;     robotdest[0].y = -1;  // Start
    robotdest[1].x = -5;     robotdest[1].y = -3;  // Point 1
    robotdest[2].x =  3;     robotdest[2].y =  7;  // Point 2
    robotdest[3].x = -3;     robotdest[3].y =  7;  // Point 3
    robotdest[4].x =  0;     robotdest[4].y =  0;  // Go to (0, 0)
    robotdest[5].x =  5;     robotdest[5].y = -3;  // Point 4
    robotdest[6].x =  0;     robotdest[6].y = 11;  // Point 5
    robotdest[7].x =  0;     robotdest[7].y = -1;  // Start


    // flag pins
    GPIO_setDir(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(OPTITRACKDATA_FROM_LINUX_BANK, OPTITRACKDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATAFORFILE_TO_LINUX_BANK, DATAFORFILE_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(LVDATA_FROM_LINUX_BANK, LVDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(LVDATA_TO_LINUX_BANK, LVDATA_TO_LINUX_FLAG, GPIO_OUTPUT);


    CLR_OPTITRACKDATA_FROM_LINUX;  // Clear = tell linux DSP is ready for new Opitrack data
    CLR_DATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new data
    CLR_DATAFORFILE_TO_LINUX;  // Clear = linux not requesting data
    SET_DATA_TO_LINUX;  // Set = put float array data into shared memory for linux
    SET_IMAGE_TO_LINUX;  // Set = put image into shared memory for linux
    CLR_LVDATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new LV data
    SET_LVDATA_TO_LINUX;  // Set = put LV char data into shared memory for linux

    // clear all possible EDMA
    EDMA3_0_Regs->SHADOW[1].ICR = 0xFFFFFFFF;

    // Add your init code here

    BIOS_start();    /* does not return */
    return(0);
}

long timecount= 0;
int whichled = 0;
// This SWI is Posted after each set of new data from the F28335
void RobotControl(void) {

    if ( new_LV_data == 1) {
        new_LV_data = 0;
        left_turn_Start_threshold = LVvalue1;
        ref_right_wall = LVvalue2;
    }

    int newOPTITRACKpose = 0;
    int i = 0;

    if (0==(timecount%1000)) {
        switch(whichled) {
        case 0:
            SETREDLED;
            CLRBLUELED;
            CLRGREENLED;
            whichled = 1;
            break;
        case 1:
            CLRREDLED;
            SETBLUELED;
            CLRGREENLED;
            whichled = 2;
            break;
        case 2:
            CLRREDLED;
            CLRBLUELED;
            SETGREENLED;
            whichled = 0;
            break;
        default:
            whichled = 0;
            break;
        }
    }

    if (GET_OPTITRACKDATA_FROM_LINUX) {

        if (new_optitrack == 0) {
            for (i=0;i<OPTITRACKDATASIZE;i++) {
                Optitrackdata[i] = ptrshrdmem->Optitrackdata[i];
            }
            new_optitrack = 1;
        }

        CLR_OPTITRACKDATA_FROM_LINUX;

    }

    if (new_optitrack == 1) {
        OPTITRACKps = UpdateOptitrackStates(ROBOTps, &newOPTITRACKpose);
        new_optitrack = 0;
    }

    // using 400deg/s gyro
    gyro = adcA3*3.0/4096.0;
    if (settlegyro < SETTLETIME) {
        settlegyro++;
        if (settlegyro < (SETTLETIME/3)) {
            // do nothing
        } else if (settlegyro < (2*SETTLETIME/3)) {
            gyro_zero = gyro_zero + gyro/(SETTLETIME/3);
        } else {
            gyro_drift += (((gyro-gyro_zero) + old_gyro)*.0005)/(SETTLETIME/3);
            old_gyro = gyro-gyro_zero;
        }
        if(settlegyro%500 == 0) {
            LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
            LCDPrintfLine(2,"");
        }
        enc1old = enc1;
        enc2old = enc2;
        newOPTITRACKpose = 0;

        SetRobotOutputs(0,0,0,0,0,0,0,0,0,0);
    }
    else {

        gyro_angle = gyro_angle - ((gyro-gyro_zero) + old_gyro)*.0005 + gyro_drift;
        old_gyro = gyro-gyro_zero;
        gyro_radians = (gyro_angle * (PI/180.0)*400.0*gyro4x_gain);

        // Kalman filtering
        vel1 = (enc1 - enc1old)/(193.0*0.001);  // calculate actual velocities
        vel2 = (enc2 - enc2old)/(193.0*0.001);
        if (fabsf(vel1) > 10.0) vel1 = vel1old; // check for encoder roll-over should never happen
        if (fabsf(vel2) > 10.0) vel2 = vel2old;
        enc1old = enc1; // save past values
        enc2old = enc2;
        vel1old = vel1;
        vel2old = vel2;

        // Step 0: update B, u
        B[0][0] = cosf(ROBOTps.theta)*0.001;
        B[1][0] = sinf(ROBOTps.theta)*0.001;
        B[2][1] = 0.001;
        u[0][0] = 0.5*(vel1 + vel2);    // linear velocity of robot
        u[1][0] = (gyro-gyro_zero)*(PI/180.0)*400.0*gyro4x_gain;    // angular velocity in rad/s (negative for right hand angle)

        // Step 1: predict the state and estimate covariance
        Matrix3x2_Mult(B, u, Bu);                   // Bu = B*u
        Matrix3x1_Add(x_pred, Bu, x_pred, 1.0, 1.0); // x_pred = x_pred(old) + Bu
        Matrix3x3_Add(P_pred, Q, P_pred, 1.0, 1.0); // P_pred = P_pred(old) + Q
        // Step 2: if there is a new measurement, then update the state
        if (1 == newOPTITRACKpose) {
            newOPTITRACKpose = 0;
            z[0][0] = OPTITRACKps.x;    // take in the LADAR measurement ?????
            z[1][0] = OPTITRACKps.y;
            // fix for OptiTrack problem at 180 degrees
            if (cosf(ROBOTps.theta) < -0.99) {
                z[2][0] = ROBOTps.theta;
            }
            else {
                z[2][0] = OPTITRACKps.theta;
            }
            // Step 2a: calculate the innovation/measurement residual, ytilde
            Matrix3x1_Add(z, x_pred, ytilde, 1.0, -1.0);    // ytilde = z-x_pred
            // Step 2b: calculate innovation covariance, S
            Matrix3x3_Add(P_pred, R, S, 1.0, 1.0);                          // S = P_pred + R
            // Step 2c: calculate the optimal Kalman gain, K
            Matrix3x3_Invert(S, S_inv);
            Matrix3x3_Mult(P_pred,  S_inv, K);                              // K = P_pred*(S^-1)
            // Step 2d: update the state estimate x_pred = x_pred(old) + K*ytilde
            Matrix3x1_Mult(K, ytilde, temp_3x1);
            Matrix3x1_Add(x_pred, temp_3x1, x_pred, 1.0, 1.0);
            // Step 2e: update the covariance estimate   P_pred = (I-K)*P_pred(old)
            Matrix3x3_Add(eye3, K, temp_3x3, 1.0, -1.0);
            Matrix3x3_Mult(temp_3x3, P_pred, P_pred);
        }   // end of correction step

        // set ROBOTps to the updated and corrected Kalman values.
        ROBOTps.x = x_pred[0][0];
        ROBOTps.y = x_pred[1][0];
        ROBOTps.theta = x_pred[2][0];

        // ================================================= BEGIN Student Code ====================================================================


        // Centered about 0deg -> 114

        front_180 = min_LADAR(28, 200);
        front_120 = min_LADAR(56, 171);
        front_90 = min_LADAR(71, 157);
        front_60 = min_LADAR(85, 142);
        front_30 = min_LADAR(100, 128);


        left_30 = min_LADAR(186, 214);
        left_50 = min_LADAR(176, 223);
        left_side = min_LADAR(114, 224);
        left_rear = min_LADAR(208,224);
        left_forward = min_LADAR(114, 200);


        right_30 = min_LADAR(14, 42);
        right_50 = min_LADAR(4, 51);
        right_side = min_LADAR(4, 113);
        right_rear = min_LADAR(4, 20);
        right_forward = min_LADAR(28, 114);


        v1_x = robotdest[statePos].x - ROBOTps.x;
        v1_y = robotdest[statePos].y - ROBOTps.y;
        v1_mag = (float)sqrt(v1_x * v1_x + v1_y * v1_y);
        v1_theta = rad2deg((float)atanf(v1_y / v1_x));
        v1_theta2 = v1_theta;

        mytheta = rad2deg(ROBOTps.theta);


        if (mytheta > 360)
            while (mytheta > 360)
                mytheta -= 360.0;
        if (mytheta < 0)
            while (mytheta < 0)
                mytheta += 360.0;

        Rv1_theta = get_adjustment_angle();

        LeftRight = cos(ROBOTps.theta) * (robotdest[statePos].y - ROBOTps.y)
                  + sin(ROBOTps.theta) * (ROBOTps.x - robotdest[statePos].x);

        // Wall following case structure
        switch (pval) {

        // Point to point
        case 1:

            // If moving directly towards robot found target
            if ( xy_control(&vref, &turn, 1.0, ROBOTps.x, ROBOTps.y,
                            robotdest[statePos].x, robotdest[statePos].y, ROBOTps.theta, 0.25, 0.5) )
            { statePos = (statePos + 1) % robotdestSize; }

            // Nothing in front or around -> keep moving to destination!
            if (front_180 >= 320) {
                pval = 1;
                break;
            }
            // Something around robot -> wall follow it
            else {
                hit_x = v1_x;
                hit_y = v1_y;
                hit_theta = v1_theta;
                hit_mag = v1_mag;
                if (left_forward <= right_forward) {
                    pval = 2;
                    tc = 0;
                    break;
                }
                if (right_forward < left_forward) {
                    pval = 3;
                    tc = 0;
                    break;
                }
            }
            break;

            // Left wall following state
            // Break out when objective is on left of robot
        case 2:
            tc++;

            // Objective is on right side of robot again
            //            if (Rv1_theta < -10 && fabsf(hit_mag - v1_mag) > 1.0) {
            //                pval = 1;
            //            }



            // Something in front
            if (front_30 <= 400) {
                ppval = 1;
                // Turn (CW) until nothing is in front
                turn = 0.003 * (3000 - front_30);
                vref = 0.1;
            }



            // Wall on left, nothing in front

            if (front_30 > 400) {// && (left_50<= 320)) {
                ppval = 2;
                turn = 0.002 * (300 - min_LADAR(198, 202));
                vref = forward_velocity * 0.7;
            }

//            if (left_30 >= 1000) {
//                turn = 0.005 * (200 - left_rear);
//                vref = forward_velocity * 0.8;
//            }



            if ((LeftRight < -1) && (tc >= 1000)){
                pval = 1;
            }

            if (v1_mag < 0.5) {
                pval = 1;
            }

            break;

            // Right wall following state
            // Break out when objective is on right of robot
        case 3:
            tc++;

            // Objective is on right side again
//            if (Rv1_theta > 10 && fabsf(hit_mag - v1_mag) > 0.75) {
//                pval = 1;
//            }

            // Something in front
            if (front_30 <= 400) {
                // Turn (CCW) until nothing is in front
                turn = 0.003 * (-3000 + front_30);
                vref = 0.1;
            }

            // Wall on right, nothing in front
            if (front_60 > 400) {//&& (right_50 <= 320)) {
                turn = 0.002 * (-300 + min_LADAR(26, 30));
                vref = forward_velocity *0.7;
            }


            //            if (right_30 >= 1000) {
            //                turn = -0.005 * (200 - right_rear);
            //                vref = forward_velocity * 0.8;
            //
            //            }

            if ((LeftRight >= 1) && (tc >= 1000)){
                pval = 1;
            }

            if (v1_mag < 0.5) {
                pval = 1;
            }

            break;
        }

        turn = MIN(turn, 5.0);
        turn = MAX(turn, -4.0);


        //==================================================== end wall following/point to point====================
        if (newLADARdata == 1) {
            newLADARdata = 0;
            for (i=0;i<228;i++) {
                LADARdistance[i] = newLADARdistance[i];
                LADARangle[i] = newLADARangle[i];
                LADARdataX[i] = newLADARdataX[i];
                LADARdataY[i] = newLADARdataY[i];

            }

        }

        if ( (timecount % 200) == 0 ) {
            LCDPrintfLine(1,"mt:%.1f,v1t:%.1f", mytheta, v1_theta);
            LCDPrintfLine(2,"Rv1:%.1f,da:%.1f", Rv1_theta, 2.3);
        }

        SetRobotOutputs(vref,turn,0,0,0,0,0,0,0,0);

        timecount++;
    }
}

pose UpdateOptitrackStates(pose localROBOTps, int * flag) {

    pose localOPTITRACKps;

    // Check for frame errors / packet loss
    if (previous_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
        frame_error++;
    }
    previous_frame = Optitrackdata[OPTITRACKDATASIZE-1];

    // Set local trackableID if first receive data
    if (firstdata){
        //trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
        trackableID = Optitrackdata[OPTITRACKDATASIZE-2];
        firstdata = 0;
    }

    // Check if local trackableID has changed - should never happen
    if (trackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
        trackableIDerror++;
        // do some sort of reset(?)
    }

    // Save position and yaw data
    if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
        // check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
        if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
            // save x,y
            // adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
            // This was the old way for Optitrack coordinates
            //localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
            //localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

            // This is the new coordinates for Motive
            localOPTITRACKps.x = -1.0*Optitrackdata[0]*FEETINONEMETER;
            localOPTITRACKps.y = Optitrackdata[1]*FEETINONEMETER+4.0;

            // make this a function
            temp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
            tempOPTITRACK_theta = Optitrackdata[2];
            if (temp_theta > 0) {
                if (temp_theta < PI) {
                    if (tempOPTITRACK_theta >= 0.0) {
                        // THETA > 0, kal in QI/II, OT in QI/II
                        localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                    } else {
                        if (temp_theta > (PI/2)) {
                            // THETA > 0, kal in QII, OT in QIII
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
                        } else {
                            // THETA > 0, kal in QI, OT in QIV
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                } else {
                    if (tempOPTITRACK_theta <= 0.0) {
                        // THETA > 0, kal in QIII, OT in QIII
                        localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
                    } else {
                        if (temp_theta > (3*PI/2)) {
                            // THETA > 0, kal in QIV, OT in QI
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + tempOPTITRACK_theta*2*PI/360.0;
                        } else {
                            // THETA > 0, kal in QIII, OT in QII
                            localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                }
            } else {
                if (temp_theta > -PI) {
                    if (tempOPTITRACK_theta <= 0.0) {
                        // THETA < 0, kal in QIII/IV, OT in QIII/IV
                        localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                    } else {
                        if (temp_theta < (-PI/2)) {
                            // THETA < 0, kal in QIII, OT in QII
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
                        } else {
                            // THETA < 0, kal in QIV, OT in QI
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                } else {
                    if (tempOPTITRACK_theta >= 0.0) {
                        // THETA < 0, kal in QI/II, OT in QI/II
                        localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
                    } else {
                        if (temp_theta < (-3*PI/2)) {
                            // THETA < 0, kal in QI, OT in QIV
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + tempOPTITRACK_theta*2*PI/360.0;
                        } else {
                            // THETA < 0, kal in QII, OT in QIII
                            localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                }
            }
            *flag = 1;
        }
    }
    return localOPTITRACKps;
}

