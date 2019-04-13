
#include <tistdtypes.h>
#include <coecsl.h>
#include "28335_spi.h"
#include "user_includes.h"

/*
 * These are variables to store values that will be used to edit and work with SPI values.
 * Used to store the encoder count values.
 */
long SPIbyte1,SPIbyte2,SPIbyte3,SPIbyte4,SPIbyte5;
long SPIenc1_reading = 0;
long SPIenc2_reading = 0;
long SPIenc3_reading = 0;
long SPIenc4_reading = 0;
int SPIenc_state = 0;
int SPIenc_state_errors = 0;

unsigned int dac1data = 0;
unsigned int dac2data = 0;

void init_SPI(void){
    /*****************************************************/
    /*********  SPI  *************************************/

    /* EALLOW at the beginning and end of this function are allowing us to edit these GPIO registers.
     * Because they are "protected".
     */
    EALLOW;

        /*
         * Configure ports 9, 10, 11, and 12 as GPIO.
         * To be used as slave select pins (http://coecsl.ece.illinois.edu/ge423/ge423_Lab4.pdf#page=13).
         * Need to set high to low to create transmission cycle.
         */
        GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
        GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
        GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;

        /*
         * GPADIR is similar to P1DIR and P2DIR on the MSP430G2553.
         * Setting a bit to 0 makes it an input, and 1 makes it an output.
         * So here, we are setting GPIO pins 9, 10, 11, and 12 as outputs
         * in order to control the slave select.
         */
        GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
        GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;

        /*
         * GPA => GPIO port A.
         * So set each of the slave select pins to high in order to start the transmission cycle.
         * Similar to MSP430G2553 P1OUT and P2OUT registers.
         */
        GpioDataRegs.GPASET.bit.GPIO9 = 1;
        GpioDataRegs.GPASET.bit.GPIO10 = 1;
        GpioDataRegs.GPASET.bit.GPIO11 = 1;
        GpioDataRegs.GPASET.bit.GPIO22 = 1;

    EDIS;

    InitSpiaGpio();

    EALLOW;
    // SS for DAC7564
        GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;  // use GPIO19 for SS
        GpioDataRegs.GPASET.bit.GPIO19 = 1;  // enabled
        GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  // GPIO19 as output
    EDIS;


    /*
     * Here we are setting the polarity and phase of the LS7366. (http://coecsl.ece.illinois.edu/ge423/datasheets/LS7366.pdf#page=1).
     * So that received bytes are shifted in most significant bit first with the leading edges of the SCK clocks.
     * Output data are shifted out on the trailing edges of the clock. The clock is generated by the TMS320x28335.
     * Also set some configurations for the SPI, such as character length,
     * interrupts for SPI, changing the baud rate, setting FIFO mode.
     */
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset

    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

    SpiaRegs.SPICCR.bit.SPICHAR = 7;   // set to transmit 8 bits

    /* Make the TMS320x28335 the master, and the chips will be the slaves.
     * Talk will be enabled so that we can transmit.
     */
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;

    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    SpiaRegs.SPISTS.all=0x00E0;

    SpiaRegs.SPIBRR = 39;   // divide by 40 2.5 Mhz

    /*
     * Resetting things that could have values in them (from previous sessions)
     * for the FIFO transmit register.
     * Also enabling interrupts for the receive buffer.
     */
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 0;

    SpiaRegs.SPIFFCT.all=0x00;

    /*
     * Now we have finished resetting things for the SPI, and we are ready to do operations.
     * Resetting things can still be done, but these should be considered more dynamic than the
     * ones done earlier.
     *
     * TXFIFO resets the FIFO when set to 1. Then the FIFO will operate normally.
     */
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;

    /*
     * Clearing out GPIO bits in order to start transmission.
     */
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    /*
     * SPITXBUF is a 16-bit register. The operation `0x20 << 8` will move the instruction bit 0x20 out 8 bits to the left.
     * This can be considered as setting the most significant byte of the register as 0x20. Specifically this instruction byte
     * read by the slave (IR) and it clears the counter. Afterwards we are setting the slave select back to high to stop the transmission.
     * Store the receive buffer to a global variable because we will use this later. All writes to the TXBUF must be left-justified.
     * This is why we are shifting the bytes over.
     *
     * Why 0x20  CLR Counter inside all LS7366 Chips
     */
    SpiaRegs.SPITXBUF = ((unsigned)0x20)<<8;     // CLR COUNT all four chips
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 1) {}  // Waiting for 1 word to be sent over
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;
    GpioDataRegs.GPASET.bit.GPIO11 = 1;
    GpioDataRegs.GPASET.bit.GPIO22 = 1;
    SPIbyte1 = SpiaRegs.SPIRXBUF;

    /*
     * Starting transmission again.  0x88 selects MDR0 and 0x83 x4 quadrature etc.
     * When writing to TXBUF this time we are communicating that we intend to write to MDR0.
     * Then we say that we are using x4 quadrature count mode, free running count mode, disabling index, and
     * setting the filter clock division output by 2.
     */
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    SpiaRegs.SPITXBUF = ((unsigned)0x88)<<8;  // WR to MDR0
    SpiaRegs.SPITXBUF = ((unsigned)0x83)<<8;
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;
    GpioDataRegs.GPASET.bit.GPIO11 = 1;
    GpioDataRegs.GPASET.bit.GPIO22 = 1;
    SPIbyte1 = SpiaRegs.SPIRXBUF;
    SPIbyte2 = SpiaRegs.SPIRXBUF;

    /*
     * Starting transmission again.
     * Now we are telling all 4 LS7366 chips that we intend to write to MDR1.
     * Then we write to MDR1 that we are using a 4-byte counter and that we enable that counter.
     */
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    SpiaRegs.SPITXBUF = ((unsigned)0x90)<<8;  // WR MDR1
    SpiaRegs.SPITXBUF = 0x00<<8;
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;
    GpioDataRegs.GPASET.bit.GPIO11 = 1;
    GpioDataRegs.GPASET.bit.GPIO22 = 1;
    SPIbyte1 = SpiaRegs.SPIRXBUF;
    SPIbyte2 = SpiaRegs.SPIRXBUF;

    /*
     * Clearing out flags, enabling interrupts.
     * Basically putting the SPI back into an expected state.
     */
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

/*********  SPI  *************************************/
/*****************************************************/

    // SPI
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt

}

void SPI_RXint(void) {
    /*
     * TODO: Postlab-Section#2-3 -> GPIO of MCP23S08 chip
     *
     * Function to read the 8-bit GPIO register after four LS7366 chips have been read
     *  - No need to poll on status RXFFST bits
     *
     */



    /*
     * Setting all GPASET bits to high means the transmission cycle is complete.
     * GPIO19 has to do with the DAC, so don't worry about it.
     */
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
    GpioDataRegs.GPASET.bit.GPIO10 = 1;
    GpioDataRegs.GPASET.bit.GPIO11 = 1;
    GpioDataRegs.GPASET.bit.GPIO22 = 1;

    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    switch (SPIenc_state) {
        case 1:
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            /*
             * Expecting 5 bytes, the instruction register and 4 bytes of the count
             * Initialize transmission to the first slave (U7).
             * Output latch count on the OTR to the transmission lines.
             *
             */
            SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
            SPIenc_state = 2;
            GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;

            break;
        case 2:
            /*
             * Slave is writing back to the SPIRXBUF.
             * We don't care about the first byte, because you can't read and write at the same time.
             * You want to AND with 0xFF to get the rightmost byte.
             * Then you combine them into a single variable in our C-code.
             */
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIenc1_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

            /*
             * Similar procedure as case 1.
             * Telling the slave (U8) to latch its encoder count to the transmit buffer.
             */
            SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
            SPIenc_state = 3;
            GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;

            break;
        case 3:
            /*
             * Slave is writing back to the SPIRXBUF.
             * We don't care about the first byte, because you can't read and write at the same time.
             * You want to AND with 0xFF to get the rightmost byte.
             * Then you combine them into a single variable in our C-code.
             */
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIenc2_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
            /*
             * Expecting 5 bytes, the 4 counts and the instruction register.
             * Initialize transmission to the first slave (U9).
             * Output latch count on the OTR to the transmission lines.
             * Make sure TXBUF is empty by setting to 0 four times.
             */
            SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
            SPIenc_state = 4;
            GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;

            break;
        case 4:
            /*
             * Slave is writing back to the SPIRXBUF.
             * We don't care about the first byte, because you can't read and write at the same time.
             * You want to AND with 0xFF to get the rightmost byte.
             * Then you combine them into a single variable in our C-code.
             */
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIenc3_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
            /*
             * Expecting 5 bytes, the 4 counts and the instruction register.
             * Initialize transmission to the first slave (U10).
             * Output latch count on the OTR to the transmission lines.
             * Make sure TXBUF is empty by setting to 0 four times.
             */
            SpiaRegs.SPIFFRX.bit.RXFFIL = 5;
            SPIenc_state = 5;
            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;

            break;
        case 5:
            /*
             * Slave is writing back to the SPIRXBUF.
             * We don't care about the first byte, because you can't read and write at the same time.
             * You want to AND with 0xFF to get the rightmost byte.
             * Then you combine them into a single variable in our C-code.
             */
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte5 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIenc4_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;

            SWI_post(&SWI_control);

            break;
        case 6:
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIenc_state = 7;

            // Output to DAC Ch2
            SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
            SPIenc_state = 7;
            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
            SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
            SpiaRegs.SPITXBUF = ((unsigned)0x12)<<8;
            SpiaRegs.SPITXBUF = (int)(dac2data << 4);
            SpiaRegs.SPITXBUF = ((int)(dac2data))<<12;

            break;
        case 7:
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte1 = SpiaRegs.SPIRXBUF;

            SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for LS7366
            SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

            SPIenc_state = 8;

            break;
        case 8:
           /*
            * read gpio register from MCP23S08
            */
           SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
           SPIenc_state = 9;
           GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;
           GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;
           GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;
           SpiaRegs.SPITXBUF = ((unsigned)0b01000001) << 8;          // [0 1 0 0 0 A1 A0 R/W] control byte address 00 read 1 CONTROL BYTE
           SpiaRegs.SPITXBUF = ((unsigned)0x09) << 8;
           SpiaRegs.SPITXBUF = 0;
           break;
        case 9:

           SPIenc_state = 0;
           SPIbyte1 = SpiaRegs.SPIRXBUF;
           SPIbyte2 = SpiaRegs.SPIRXBUF;
           SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
           GPIOreading = (SPIbyte3);
           break;

        default:
            SPIbyte1 = SpiaRegs.SPIRXBUF;  // Should never get in here.
            SPIbyte2 = SpiaRegs.SPIRXBUF;
            SPIbyte3 = SpiaRegs.SPIRXBUF;
            SPIbyte4 = SpiaRegs.SPIRXBUF;
            SPIbyte5 = SpiaRegs.SPIRXBUF;
            SPIenc_state_errors++;
            break;
    }

    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE

}

void start_SPI(void) {

    /*
     * Starting transmission for LS7366. Loading count to OTR.
     * OTR is a register which can be read out on the MISO  later.
     *
     * selecting all 4 LS7366 chips
     */

    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
    SpiaRegs.SPITXBUF = ((unsigned)0xE8)<<8; // Latch All ENCs
    SPIenc_state = 1;
}


/******* Start Student Code ****/


void Init_MCP23S08 (unsigned int IODIRvalue, unsigned int IPOLvalue,
                    unsigned int GPPUvalue,  unsigned int IOCONvalue, unsigned int OLATvalue) {
    /*
     * Initialize the MCP23S08
     *   - Function of the parameters passed
     * Poll on the RXFFST status bits
     *   - Determines when SPI transmit/receive have finished -> then return
     *
     */
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0; //set to coincide with chip polarity
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;

    GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;                     // Set GPIO 58 48 and 49 low to select MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;                     // Set GPIO 58 48 and 49 low to select MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;                     // Set GPIO 58 48 and 49 low to select MCP23S08

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W] control byte address 00 write 0 CONTROL BYTE
    SpiaRegs.SPITXBUF = ((unsigned)0x00) << 8                 // ADRESSING IODIR 0x00
    SpiaRegs.SPITXBUF = IODIRvalue;                           // Whatever we want to set it to

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W]
    SpiaRegs.SPITXBUF = ((unsigned)0x01) << 8                 // IPOLvalue address 0x01
    SpiaRegs.SPITXBUF = IPOLvalue;                            // Whatever we want to set it to

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W]
    SpiaRegs.SPITXBUF = ((unsigned)0x06) << 8                 // GPPUvalue address 0x06
    SpiaRegs.SPITXBUF = GPPUvalue;                            // Whatever we want to set it to

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W]
    SpiaRegs.SPITXBUF = ((unsigned)0x05) << 8                 // IOCONvalue address 0x05
    SpiaRegs.SPITXBUF = IOCONvalue;                           // Whatever we want to set it to

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W]
    SpiaRegs.SPITXBUF = ((unsigned)0x0A) << 8                 // OLATvalue address 0x0A
    SpiaRegs.SPITXBUF = OLATvalue;                            // Whatever we want to set it to


    while (SpiaRegs.SPIFFRX.bit.RXFFST != 15) {} // Poll on RXFFST status bits

    GpioDataRegs.GPASET.bit.GPIO58 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO48 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO49 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08

}


void SetPortLatch(unsigned int byte) {
    /*
     * LSB to MCP23S08's OLAT register
     *   - Only output bits will be changed by writing to OLAT register
     * Poll on RXFFST status bits
     *   - Determines when SPI transmit/receive have finished -> then return
     *
     */
    GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08

    SpiaRegs.SPITXBUF = ((unsigned)0b01000000) << 8;          // [0 1 0 0 0 A1 A0 R/W]
    SpiaRegs.SPITXBUF = ((unsigned)0x09) << 8;                // GPIO register address 0x09 writing to gpio changes the olat register values latching them to the outputs
    SpiaRegs.SPITXBUF = byte;                            // whatever we feed to the function


    while (SpiaRegs.SPIFFRX.bit.RXFFST != 3) {} // Poll on RXFFST status bits

    GpioDataRegs.GPASET.bit.GPIO58 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO48 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO49 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
}

unsigned int ReadPort(void) {
    /*
     * Read and return 8-bit value from GPIO register.
     * Poll on RXFFST status bits
     *   - Determines when SPI transmit/receive have finished -> then return
     *
     */
    GpioDataRegs.GPACLEAR.bit.GPIO58 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO48 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08
    GpioDataRegs.GPACLEAR.bit.GPIO49 = 1;                     // Set GPIO 58 48 and 49 low to set Y0 low and start transmission cycle MCP23S08

    SpiaRegs.SPITXBUF = ((unsigned)0b01000001) << 8;          // [0 1 0 0 0 A1 A0 R/W] control byte address 00 read 1 CONTROL BYTE
    SpiaRegs.SPITXBUF = ((unsigned)0x09) << 8;                // GPIO register address 0x09

    while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {} // Poll on RXFFST status bits

    GpioDataRegs.GPASET.bit.GPIO58 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO48 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
    GpioDataRegs.GPASET.bit.GPIO49 = 1;                       // Set GPIO 58 48 and 49 high to get Y0 to go high and end transmission cycle MCP23S08
}


/* SECOND CODING EXERCISE (for postlab) -> in SPI_RXint(void) */


void writeDAC7564(float dac1,float dac2) {

    int rawdac1,rawdac2;

    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;  // set for DAC7564
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;

    rawdac1 = (int) (dac1 * 1638 + 0.5); // The 0.5 is used to round to the nearest integer
    rawdac2 = (int) (dac2 * 1638 + 0.5); // The 0.5 is used to round to the nearest integer


    if (rawdac1 < 0) rawdac1 = 0;
    if (rawdac1 > 4095) rawdac1 = 4095;
    if (rawdac2 < 0) rawdac2 = 0;
    if (rawdac2 > 4095) rawdac2 = 4095;

    dac1data = rawdac1;
    dac2data = rawdac2;

    // Output to DAC Ch1
    SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
    SPIenc_state = 6;
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIL = 3;
    SpiaRegs.SPITXBUF = ((unsigned)0x10)<<8;
    SpiaRegs.SPITXBUF = (int)(dac1data << 4);
    SpiaRegs.SPITXBUF = ((int)(dac1data))<<12;

}