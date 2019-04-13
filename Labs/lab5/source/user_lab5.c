/*
 *  ======== main.c ========
 */

#include <coecsl.h>
#include "28335_dma.h"
#include "28335_spi.h"
#include "28335_inits.h"
#include "mcbsp_com.h"
#include "i2c.h"
#include "user_PIFuncs.h"
#include "user_include.h"



#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/knl/Task.h>



extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;


void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}


void updateData(void);
void sendData(void);

extern volatile int new_irdata_i2c;
extern int adc1_i2c;
extern int adc2_i2c;
extern int adc3_i2c;
extern int adc4_i2c;
extern int adc5_i2c;
extern int adc6_i2c;
extern int adc7_i2c;
extern int adc8_i2c;
extern int CompassNew;
extern int ir1_i2c,ir2_i2c,ir3_i2c,ir4_i2c,ir5_i2c;

extern int SPIenc_state;
extern long SPIenc1_reading;
extern long SPIenc2_reading;
extern long SPIenc3_reading;
extern long SPIenc4_reading;

extern unsigned int nocomm;
extern int newOMAPdata;
extern int PIVEL_enable;
extern float mcbsp_vref;
extern float mcbsp_turn;
extern float mcbsp_pwm1;
extern float mcbsp_pwm2;
extern float mcbsp_dac1;
extern float mcbsp_dac2;
extern long McBSP_COMerr;
extern int McBSP_RecGoodData_ReadyTX;
extern long McBSPb_int_count;
extern int McBSPB_rdy;
extern mcbsp28x_com TXBuff;
extern mcbspL138_com RXBuff;

unsigned long timeint = 0;
unsigned long noi2c = 0;

float gyro_x = 0;
float gyro_y = 0;

float Enc1_rad = 0;
float Enc2_rad = 0;
float Enc3_rad = 0;
float Enc4_rad = 0;

int switchstate = -2;

int newF28335_Extra = 0;
float F28335_Extra1 = 0.0;
float F28335_Extra2 = 0.0;
float F28335_Extra3 = 0.0;
float F28335_Extra4 = 0.0;

int raw_adc_A0 = 0;
int raw_adc_A1 = 0;
int raw_adc_A2 = 0;
int raw_adc_A3 = 0;
int raw_adc_A4 = 0;
int raw_adc_A5 = 0;
int raw_adc_A6 = 0;
int raw_adc_A7 = 0;
int raw_adc_B0 = 0;
int raw_adc_B1 = 0;
int raw_adc_B2 = 0;
int raw_adc_B3 = 0;
int raw_adc_B4 = 0;
int raw_adc_B5 = 0;
int raw_adc_B6 = 0;
int raw_adc_B7 = 0;

float omap_vref = 0;
float omap_turn = 0;
float omap_pwm1 = 0;
float omap_pwm2 = 0;
float omap_dac1 = 0;
float omap_dac2 = 0;
int omap_PIVEL_enable = 0;

eqep_t enc1;
eqep_t enc2;
float value_enc1 = 0;
float value_enc2 = 0;

//Student inits
// Student variables
float u1 = 0;
float u2 = 0;

unsigned long time_elapsed_ms = 0;
float p_current_L = 0.0;  // Left position
float p_current_R = 0.0;  // Right position
float p_old_L = 0.0;
float p_old_R = 0.0;
float v_L = 0;
float v_R = 0;
float Vpos = 2.1561 * 0.6;
float Vneg = 2.5251 * 0.6;
float Cpos = 2.8602 * 0.6;
float Cneg = -2.3807 * 0.6;
float K_p = 3.0;
//float K_I = 5.0;
float K_I = 6.0;
float Kp_turn = 3.0;


float vL_desired = 0;  // Vref
float vR_desired = 0;  // Vref
float eL = 0;
float eR = 0;
float eR_out1 = 0;
float eR_out2 = 0;
float eL_out1 = 0;
float eL_out2 = 0;
float old_eR = 0;
float old_eL = 0;
float inte_eR = 0;
float inte_eL = 0;
float old_inte_eR = 0;
float old_inte_eL = 0;
float Kp_turn_out = 0;
float turn_command = 0;  // Enc3_rad/100

/*
 *  ======== main ========
 */
Int main()
{ 
    // disable interrupts
    DINT;
    IER = 0x0000;
    IFR = 0x0000;

    unsigned long delay;
    // load code from flash memory into ram

    MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);

    // initialize system (clocks, etc.)
    InitSysCtrl();
//    InitPll(PLL,2);.//./

    // Initialize flash memory for 3 wait states.  Programmers Note:  We modified TI's default function to 3 wait states
    InitFlash();

    // power-up ADCs
    AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;
    for (delay = 1000000L; delay; delay--) nop();
    AdcRegs.ADCTRL3.bit.ADCPWDN = 1;
    for (delay = 1000L; delay; delay--) nop();

    // reconfigure clocks
    EALLOW;
    SysCtrlRegs.HISPCP.all = 0x0000;
    SysCtrlRegs.LOSPCP.all = 0x0000;
    EDIS;


    //  Init and zero encoders
    init_EQEP(&enc1, EQEP1, 3000, 1, 0.0);
    init_EQEP(&enc2, EQEP2, 3000, -1, 0.0);
    EQep1Regs.QPOSCNT = 0;
    EQep2Regs.QPOSCNT = 0;

    // Initialize PWMs
    init_PWM(EPWM1);
    init_PWM(EPWM2);
    init_PWM_AS_RCSERVO(EPWM3B);  // sets up EPWM3A and EPWM3B for RCservo
    init_PWM_AS_RCSERVO(EPWM4B); // sets up EPWM4A and EPWM4B for RCservo
    init_PWM_AS_RCSERVO(EPWM5);  // sets up EPWM5A for RCservo

    // System initializations
    pre_init();

    // initialize serial port A to 115200 baud
    init_serial(&SerialA,115200,NULL);
    // initialize serial port B to 57600 baud
    init_serial(&SerialB,57600,NULL);
    // initialize serial port C to 19200 baud
    init_serial(&SerialC,19200,NULL);

    EALLOW;  // set up LED GPIOs
        GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
        GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
        GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
        GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

        // set up GPIO3 for amp enable or disable
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    EDIS;

    init_dma_mcbsp( (Uint32)&TXBuff.darray[0],(Uint32)&RXBuff.darray[0]);
    InitMcbspbGpio();
    InitMcbspb();
    InitMcbspb32bit();
    init_SPI();
    InitI2CGpio();
    Init_i2c();

    // Add your inits here

    // Finalize inits and start DMA/McBSP
    post_init();

    BIOS_start();    /* does not return */
    return(0);
}

void start_dataCollection(void) {

 	// Start SPI
	start_SPI();
	
}

void control(void) {

    updateData();


    // ===== STUDENT CODE =====

    time_elapsed_ms++;

    p_current_L = Enc1_rad;  // in radians
    p_current_R = Enc2_rad;

    v_L = (p_current_L - p_old_L) / 0.001;
    v_R = (p_current_R - p_old_R) / 0.001;

    v_L /= 194.0;  // convert to tiles/sec
    v_R /= 194.0;

    switch (switchstate) {
        case 0:  // All switches are forward
            // Print "Robot off"
            if ((time_elapsed_ms % 100) == 0)
                LCDPrintfLine(1, "Robot off.");
            u1 = 0;
            u2 = 0;
            PWM_out(EPWM1, u1);
            PWM_out(EPWM2, -u2);  // change direction
            break;
        case 0x01:
            if ((time_elapsed_ms % 100) == 0) {
                LCDPrintfLine(1, "OL,u1=%.1f,u2=%.1f", u1, u2);
                LCDPrintfLine(2, "vL=%.1f,vR=%.1f", v_L, v_R);
            }
            u1 = Enc3_rad / 100;
            u2 = Enc3_rad / 100;

            if (u1 < -10)
                u1 = -10;
            if (u1 > 10)
                u1 = 10;
            if (u2 < -10)
                u2 = -10;
            if (u2 > 10)
                u2 = 10;

            PWM_out(EPWM1, u1);
            PWM_out(EPWM2, -u2);  // change direction
            break;


        case 0x02:  // Onlyvel switch 2 is on
            vL_desired = Enc3_rad/100;  // Vref
            vR_desired = Enc3_rad/100;  // Vref

            eL = vL_desired - v_L;
            eR = vR_desired - v_R;

            // === Right side ===
            eR_out1 = eR * K_p;  // top arrow

            // Integrator step
            inte_eR = old_inte_eR + (old_eR + eR) / 2 * .001;

            old_eR = eR;    // store eR

            eR_out2 = inte_eR * K_I;    // multiply integral by K_I

            // node1 add both errors
            u2 = eR_out1 + eR_out2;

            // friction_comp / node2
            if (v_R > 0)
                u2 += Vpos*v_R + Cpos;
            else
                u2 += Vneg*v_R + Cneg;

            if (fabs(u2) > 10)
                inte_eR *= 0.99;
            // right_motor
            old_inte_eR = inte_eR;  //store int

            // === Left side ===
            eL_out1 = eL * K_p;  // top arrow

            // Integrator step
            inte_eL = old_inte_eL + (((old_eL + eL) / 2) * .001);

            old_eL = eL;    // store eL

            eL_out2 = inte_eL * K_I;    // multiply integral by K_I

            // node1 add both errors
            u1 = eL_out1 + eL_out2;

            // friction_comp
            if (v_L > 0)
                u1 += Vpos*v_L + Cpos;
            else
                u1 += Vneg*v_L + Cneg;

            if (fabs(u1) > 10)
                inte_eL *= 0.99;

            old_inte_eL = inte_eL;  //store int

            // u theresholding
            if (u1 < -10)
                u1 = -10;
            if (u1 > 10)
                u1 = 10;
            if (u2 < -10)
                u2 = -10;
            if (u2 > 10)
                u2 = 10;

            PWM_out(EPWM1, u1);
            PWM_out(EPWM2, -u2);

            if ((time_elapsed_ms % 100) == 0) {
                LCDPrintfLine(1, "CL,%.1f", vL_desired);
                LCDPrintfLine(2, "%.1f, %.1f", v_L, v_R);
            }

            break;

        case 0x04:  // Only third switch is back
            vL_desired = Enc3_rad/100;  // Vref
            vR_desired = Enc3_rad/100;  // Vref

            eL = vL_desired - v_L;
            eR = vR_desired - v_R;

            eR -= Kp_turn_out;
            eL += Kp_turn_out;

            // === Right side ===
            eR_out1 = eR * K_p;  // top arrow

            // Integrator step
            inte_eR = old_inte_eR +(((old_eR + eR) / 2) * .001);

            old_eR = eR;    // store eR

            eR_out2 = inte_eR * K_I;    // multiply integral by K_I

            // node1 add both errors
            u2 = eR_out1 + eR_out2;

            // friction_comp / node2
            if (v_R > 0)
                u2 += Vpos*v_R + Cpos;
            else
                u2 += Vneg*v_R + Cneg;

            if (fabs(u2) > 10)
                inte_eR *= 0.99;
            // right_motor
            old_inte_eR = inte_eR;  //store int

            // === Left side ===
            eL_out1 = eL * K_p;  // top arrow

            // Integrator step
            inte_eL = old_inte_eL + (((old_eL + eL) / 2) * .001);

            old_eL = eL;    // store eL

            eL_out2 = inte_eL * K_I;    // multiply integral by K_I

            // node1 add both errors
            u1 = eL_out1 + eL_out2;

            // friction_comp
            if (v_L > 0)
                u1 += Vpos*v_L + Cpos;
            else
                u1 += Vneg*v_L + Cneg;

            if (fabs(u1) > 10)
                inte_eL *= 0.99;

            old_inte_eL = inte_eL;  //store int

            // u theresholding
            if (u1 < -10)
                u1 = -10;
            if (u1 > 10)
                u1 = 10;
            if (u2 < -10)
                u2 = -10;
            if (u2 > 10)
                u2 = 10;

            Kp_turn_out = Kp_turn * (v_R - v_L);

            PWM_out(EPWM1, u1);
            PWM_out(EPWM2, -u2);

            if ((time_elapsed_ms % 100) == 0) {
                LCDPrintfLine(1, "CPI,%.1f", vL_desired);
                LCDPrintfLine(2, "%.1f, %.1f", v_L, v_R);
            }

            break;

        case 0x08:  // Only fourth switch is back
            vL_desired = 1;  // Vref
            vR_desired = 1;  // Vref

            turn_command = Enc3_rad/100;

            eL = vL_desired - v_L;
            eR = vR_desired - v_R;

            eR -= Kp_turn_out;
            eL += Kp_turn_out;

            // === Right side ===
            eR_out1 = eR * K_p;  // top arrow

            // Integrator step
            inte_eR = old_inte_eR +(((old_eR + eR) / 2) * .001);

            old_eR = eR;    // store eR

            eR_out2 = inte_eR * K_I;    // multiply integral by K_I

            // node1 add both errors
            u2 = eR_out1 + eR_out2;

            // friction_comp / node2
            if (v_R > 0)
                u2 += Vpos*v_R + Cpos;
            else
                u2 += Vneg*v_R + Cneg;

            if (fabs(u2) > 10)
                inte_eR *= 0.99;
            // right_motor
            old_inte_eR = inte_eR;  //store int

            // === Left side ===
            eL_out1 = eL * K_p;  // top arrow

            // Integrator step
            inte_eL = old_inte_eL + (((old_eL + eL) / 2) * .001);

            old_eL = eL;    // store eL

            eL_out2 = inte_eL * K_I;    // multiply integral by K_I

            // node1 add both errors
            u1 = eL_out1 + eL_out2;

            // friction_comp
            if (v_L > 0)
                u1 += Vpos*v_L + Cpos;
            else
                u1 += Vneg*v_L + Cneg;

            if (fabs(u1) > 10)
                inte_eL *= 0.99;

            old_inte_eL = inte_eL;  //store int

            Kp_turn_out = Kp_turn * (v_R - v_L + turn_command);

            // u theresholding
            if (u1 < -10)
                u1 = -10;
            if (u1 > 10)
                u1 = 10;
            if (u2 < -10)
                u2 = -10;
            if (u2 > 10)
                u2 = 10;

            // More thresholding
            if (Kp_turn_out < -3)
                Kp_turn_out = -3;
            else if (Kp_turn_out > 3)
                Kp_turn_out = 3;

            PWM_out(EPWM1, u1);
            PWM_out(EPWM2, -u2);

            if ((time_elapsed_ms % 100) == 0) {
                LCDPrintfLine(1, "Closed PI, turn=%.1f", turn_command);
                LCDPrintfLine(2, "vL=%.1f, vR=%.1f", v_L, v_R);
            }

            break;


        default:
             // Print "Robot off"
             if ((time_elapsed_ms % 100) == 0)
                 LCDPrintfLine(1, "Robot off..");
             u1 = 0;
             u2 = 0;
             PWM_out(EPWM1, u1);
             PWM_out(EPWM2, -u2);  // change direction
             break;

    }


    p_old_L = p_current_L;
    p_old_R = p_current_R;

//
//    float dac1_val = (Enc1_rad + 10) / 8.0;
//    float dac2_val = (Enc2_rad + 10) / 8.0;
//
//    writeDAC7564(dac1_val, dac2_val);

    // y 0, angle 0
    // xpos, velocity

    F28335_Extra1 = p_current_L/194;  // sending to LabView
    F28335_Extra2 = 0;  // "
    F28335_Extra3 = v_L;
    F28335_Extra4 = 0;

    // omap_dac1 and omap_dac2 have been recieved from LabView
//    writeDAC7564(omap_dac1, omap_dac2);

//
//    if ((time_elapsed_ms % 100) == 0) {
//        LCDPrintfLine(1, "v_L %.1f, v_R %.1f", v_L, v_R);
//        LCDPrintfLine(2, "u1: %.1f, u2: %.1f", u1, u2);
//        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//    }
//
    // ===== STUDENT CODE =====

    sendData();

    timeint++;
}


void updateData(void) {

    value_enc1 = EQEP_read(&enc1);
    value_enc2 = EQEP_read(&enc2);

    Enc1_rad = ((float) SPIenc1_reading)*(TWOPI/(2000.0f)); // left encoder
    Enc1_rad = -Enc1_rad;
    Enc2_rad = ((float) SPIenc2_reading)*(TWOPI/(2000.0f)); // right encoder
    Enc3_rad = ((float) SPIenc3_reading)*(TWOPI/(2000.0f));
    Enc4_rad = ((float) SPIenc4_reading)*(TWOPI/(2000.0f));


    switchstate = (int)((0x000F & ~(((GpioDataRegs.GPBDAT.all & 0x0C000000)>>24) | ((GpioDataRegs.GPBDAT.all & 0x00030000)>>16))));

    if (newOMAPdata) {
        omap_vref = mcbsp_vref;
        omap_turn = mcbsp_turn;
        omap_pwm1 = mcbsp_pwm1;
        omap_pwm2 = mcbsp_pwm2;
        omap_dac1 = mcbsp_dac1;
        omap_dac2 = mcbsp_dac2;
        omap_PIVEL_enable = PIVEL_enable;
        newOMAPdata = 0;
    }

}

void sendData(void) {

    if (McBSPB_rdy == 1) {
        TXBuff.data.validcode = VALID_28XTO67X;
        TXBuff.data.DSC_ENC[0] = value_enc1;
        TXBuff.data.DSC_ENC[1] = value_enc2;
        TXBuff.data.SPI_ENC[0] = (float) Enc1_rad;
        TXBuff.data.SPI_ENC[1] = (float) Enc2_rad;
        TXBuff.data.SPI_ENC[2] = (float) Enc3_rad;
        TXBuff.data.SPI_ENC[3] = (float) Enc4_rad;
        TXBuff.data.compass = (float)CompassNew;
        TXBuff.data.switchstate = (float)switchstate;
        TXBuff.data.ADC[0] = (float)raw_adc_A0;
        TXBuff.data.ADC[1] = (float)raw_adc_B0;
        TXBuff.data.ADC[2] = (float)raw_adc_A1;
        TXBuff.data.ADC[3] = (float)raw_adc_B1;
        TXBuff.data.ADC[4] = (float)raw_adc_A2;
        TXBuff.data.ADC[5] = (float)raw_adc_B2;
        TXBuff.data.ADC[6] = (float)raw_adc_A3;
        TXBuff.data.ADC[7] = (float)raw_adc_B3;
        TXBuff.data.ADC[8] = (float)raw_adc_A4;
        TXBuff.data.ADC[9] = (float)raw_adc_B4;
        TXBuff.data.ADC[10] = (float)raw_adc_A5;
        TXBuff.data.ADC[11] = (float)raw_adc_B5;
        TXBuff.data.ADC[12] = (float)raw_adc_A6;
        TXBuff.data.ADC[13] = (float)raw_adc_A7;
        TXBuff.data.ADC[14] = 0; // Reserved   was (float)atmel_adc1;
        TXBuff.data.ADC[15] = 0; // Reserved   was (float)atmel_adc2;
        TXBuff.data.ADC[16] = 0; // Reserved   was (float)atmel_adc3;
        TXBuff.data.ADC[17] = 0; // Reserved   was (float)atmel_adc4;
        TXBuff.data.IR[0] = 0; // Reserved   was (float)atmel_ir1;
        TXBuff.data.IR[1] = 0; // Reserved   was (float)atmel_ir2;
        TXBuff.data.IR[2] = 0; // Reserved   was (float)atmel_ir3;
        TXBuff.data.IR[3] = 0; // Reserved   was (float)atmel_ir4;
        TXBuff.data.IR[4] = 0; // Reserved   was (float)atmel_ir5;
        TXBuff.data.GYRO[0] = (float)gyro_x;
        TXBuff.data.GYRO[1] = (float)gyro_y;
        TXBuff.data.F28335_EXTRA[0] = (float)F28335_Extra1;
        TXBuff.data.F28335_EXTRA[1] = (float)F28335_Extra2;
        TXBuff.data.F28335_EXTRA[2] = (float)F28335_Extra3;
        TXBuff.data.F28335_EXTRA[3] = (float)F28335_Extra4;

        if (McBSP_RecGoodData_ReadyTX == 1) {
            McBSPB_rdy = 0;
            start_dma();
            McBSP_RecGoodData_ReadyTX = 0;
        }
    }
}

