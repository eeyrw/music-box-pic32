#include <p32xxxx.h>
#include <plib.h>
/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "HardwareProfile.h"
#include "./USB/usb_function_midi.h"
#include "USB/usb_function_cdc.h"
#include "BspInit.h"

short buffer_a[BUFFER_LENGTH];
short buffer_b[BUFFER_LENGTH];

volatile unsigned char bufferAFull = 0;
volatile unsigned char bufferBFull = 0;

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
void InitializeSystem(void)
{

    // Configure the PIC32 core for the best performance
    // at the operating frequency. The operating frequency is already set to
    // 60MHz through Device Config Registers
    SYSTEMConfigPerformance(60000000);

    //	The USB specifications require that USB peripheral devices must never source
    //	current onto the Vbus pin.  Additionally, USB peripherals should not source
    //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
    //	When designing a self powered (as opposed to bus powered) USB peripheral
    //	device, the firmware should make sure not to turn on the USB module and D+
    //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
    //	firmware needs some means to detect when Vbus is being powered by the host.
    //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
    // 	can be used to detect when Vbus is high (host actively powering), or low
    //	(host is shut down or otherwise not supplying power).  The USB firmware
    // 	can then periodically poll this I/O pin to know when it is okay to turn on
    //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
    //	peripheral device, it is not possible to source current on D+ or D- when the
    //	host is not actively providing power on Vbus. Therefore, implementing this
    //	bus sense feature is optional.  This firmware can be made to use this bus
    //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
    //	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

    //	If the host PC sends a GetStatus (device) request, the firmware must respond
    //	and let the host know if the USB peripheral device is currently bus powered
    //	or self powered.  See chapter 9 in the official USB specifications for details
    //	regarding this request.  If the peripheral device is capable of being both
    //	self and bus powered, it should not return a hard coded value for this request.
    //	Instead, firmware should check if it is currently self or bus powered, and
    //	respond accordingly.  If the hardware has been configured like demonstrated
    //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
    //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
    //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
    //	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin
    //  has been mapped	to it.
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
                     //variables to known states.

    U1RXR = 0b0011; //UART1 RX ON PB3
    RPB0R = 0b0100; //SPI2 CS ON PB0
    RPA0R = 0b0011; //SPI1 CS (LRCLK for I2S) on PA0
    RPB1R = 0b0011; //SPI1 SDO (DIN for I2S) on PB1
    RPC5R = 0b0001; //UART1 TX on PC5
    RPA9R = 0b0100; //SPI2 SDO on PA9
    SDI2R = 0b0010; //SPI2 SDI on PA4
    RPA8R = 0b0101; //PWM OC2 on PA8
    RPC0R = 0b0101; //PWM OC1 on PC0

    /* Open Timer2 with Period register value */
    OpenTimer2(T2_ON, 0x550);

    /* Enable OC | 32 bit Mode  | Timer2 is selected | Continuous O/P   | OC Pin High , S Compare value, Compare value*/
    OpenOC1(OC_ON | OC_TIMER_MODE32 | OC_TIMER2_SRC | OC_CONTINUE_PULSE | OC_LOW_HIGH, 0x550, 0x100);
    OpenOC2(OC_ON | OC_TIMER_MODE32 | OC_TIMER2_SRC | OC_CONTINUE_PULSE | OC_LOW_HIGH, 0x550, 0x500);
    /*
      The expected output looks like the diagram below with approximately 6% duty-cycle

           ||      ||      ||
     ______||______||______||______
     
*/
    TRISBbits.TRISB15 = 0;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB4 = 0;
    PORTBbits.RB4 = 1;
    INTEnableSystemMultiVectoredInt();

    // Fill all buffers first at start.
    generate_sine(&buffer_a[0]);
    generate_sine(&buffer_b[0]);

    delay_ms(5);

    init_i2s1();
    i2s_init_DMA();

    // Trigger the DMA to start the transfer by switching the SPI1 transmit complete interrupt flag up.
    IFS1bits.SPI1TXIF = 1;

} //end InitializeSystem

void init_i2s1()
{
    // http://chipkit.net/forum/viewtopic.php?f=6&t=3137&start=10
    /* The following code example will initialize the SPI1 Module in I2S Master mode. */
    /* It assumes that none of the SPI1 input pins are shared with an analog input. */
    unsigned int rData;
    IEC0CLR = 0x03800000; // disable all interrupts
    IFS1bits.SPI1TXIF = 0;
    SPI1CON = 0;     // Stops and resets the SPI1.
    SPI1CON2 = 0;    // Reset audio settings
    SPI1BRG = 0;     // Reset Baud rate register
    rData = SPI1BUF; // clears the receive buffer

    SPI1STATCLR = 0x40;      // clear the Overflow
    SPI1CON2 = 0x00000080;   // I2S Mode, AUDEN = 1, AUDMON = 0
    SPI1CON2bits.IGNROV = 1; // Ignore Receive Overflow bit (for Audio Data Transmissions)
    SPI1CON2bits.IGNTUR = 1; //  Ignore Transmit Underrun bit (for Audio Data Transmissions) 1 = A TUR is not a critical error and zeros are transmitted until thSPIxTXB is not empty 0 = A TUR is a critical error which stop SPI operation

    SPI1CONbits.ENHBUF = 1; // 1 = Enhanced Buffer mode is enabled
    SPI1CON = 0x00000060;   // Master mode, SPI ON, CKP = 1, 16-bit audio channel
    SPI1CONbits.STXISEL = 0b11;
    SPI1CONbits.DISSDI = 1; // 0 = Disable SDI bit
    SPI1CONSET = 0x00008000;

    IFS1CLR = 0x000000f0;
    IPC7CLR = 0x1F000000;
    IPC7SET = 0x1C000000;

    // REFCLK is used by the Baud Rate Generator
    SPI1CONbits.MCLKSEL = 1;

    // 16-bit Data, 16-bit FIFO, 32-bit Channel/64-bit Frame
    SPI1CONbits.MODE32 = 0;
    SPI1CONbits.MODE16 = 1;
    // Baud Rate Generator
    SPI1BRG = 1;

    IEC1bits.SPI1TXIE = 0;

    // data, 32 bits per frame
    // from here, the device is ready to receive and transmit data
    /* Note: A few of bits related to frame settings are not required to be set in the SPI1CON */
    /* register during audio mode operation. Please refer to the notes in the SPIxCON2 register.*/
    PPSUnLock;
    //PPSOutput(3, RPA4, REFCLKO);	// MCLK
    PPSOutput(1, RPA0, SS1);  // LRCLK
    PPSOutput(2, RPB1, SDO1); // SDATA
    // RB14 is BCLK
    PPSLock;

    REFOCONbits.OE = 0;
    REFOCONbits.ON = 0;
    REFOCONbits.ROSEL = 6; // Using 96M USB PLL CLock
                           // This value from AN1422
                           // 48 KHz (12.2880MHz)
                           // REFOCONbits.RODIV = 3;
                           // REFOTRIM = 464<<23;
                           // 44.1 KHz (11.28893MHz)
                           //    REFOCONbits.RODIV = 4;
                           //    REFOTRIM = 128<<23;
                           // 32 KHz (8.1920MHz)
    REFOCONbits.RODIV = 5;
    REFOTRIM = 440 << 23;
    REFOCONSET = 0x00000200;
    REFOCONbits.OE = 1;
    REFOCONbits.ON = 1;
}

void i2s_init_DMA(void)
{
    DMACONCLR = 0x8000; // disable entire DMA.
    IEC1bits.DMA0IE = 1;
    IFS1bits.DMA0IF = 0;
    IPC10bits.DMA0IP = 7; // Setting DMA0 at highest priority.
    IPC10bits.DMA0IS = 3; // Setting DMA0 at highest sub-priority.
    DMACONSET = 0x8000;   // enable DMA.
    DCH0CON = 0x0000;
    DCRCCON = 0x00;         //
    DCH0INTCLR = 0xff00ff;  // clear DMA0 interrupts register.
    DCH0INTbits.CHSDIE = 1; // DMA0 Interrupts when source done enabled.
    DCH0ECON = 0x00;
    DCH0SSA = KVA_TO_PA(&buffer_a[0]); // DMA0 source address.
    DCH0DSA = KVA_TO_PA(&SPI1BUF);     // DMA0 destination address.

    DCH0SSIZ = BUFFER_LENGTH * 2; // DMA0 Source size (default).
    DCH0DSIZ = 2;                 // DMA0 destination size.
    DCH0CSIZ = 2;                 // DMA0 cell size.

    DCH0ECONbits.CHSIRQ = _SPI1_TX_IRQ; // DMA0 transfer triggered by which interrupt? (On PIC32MX - it is by _IRQ suffix!)
    DCH0ECONbits.AIRQEN = 0;            // do not enable DMA0 transfer abort interrupt.
    DCH0ECONbits.SIRQEN = 1;            // enable DMA0 transfer start interrupt.
    DCH0CONbits.CHAEN = 0;              // DMA Channel 0 is always disabled right after the transfer.
    DCH0CONbits.CHEN = 1;               // DMA Channel 0 is enabled.

    IEC1bits.DMA1IE = 1;
    IFS1bits.DMA1IF = 0;
    IPC10bits.DMA1IP = 7; // Setting DMA1 at highest priority.
    IPC10bits.DMA1IS = 3; // Setting DMA1 at highest sub-priority.
    DCH1CON = 0x0000;
    DCH1INTCLR = 0xff00ff;  // clear DMA1 interrupts register.
    DCH1INTbits.CHSDIE = 1; // DMA1 Interrupts when source done enabled.
    DCH1ECON = 0x00;
    DCH1SSA = KVA_TO_PA(&buffer_b[0]); // DMA1 source address.
    DCH1DSA = KVA_TO_PA(&SPI1BUF);     // DMA1 destination address.

    DCH1SSIZ = BUFFER_LENGTH * 2; // DMA1 Source size (default).
    DCH1DSIZ = 2;                 // DMA1 destination size.
    DCH1CSIZ = 2;                 // DMA1 cell size.

    DCH1ECONbits.CHSIRQ = _SPI1_TX_IRQ; // DMA1 transfer triggered by which interrupt? (On PIC32MX - it is by _IRQ suffix!)
    DCH1ECONbits.AIRQEN = 0;            // do not enable DMA1 transfer abort interrupt.
    DCH1ECONbits.SIRQEN = 1;            // enable DMA1 transfer start interrupt.
    DCH1CONbits.CHAEN = 0;              // DMA Channel 1 is always disabled right after the transfer.
    DCH1CONbits.CHEN = 0;               // DMA Channel 1 is enabled.
}

void delay_ms(unsigned int count)
{
    T1CON = 0x8030; // turn on timer, prescaler to 256 (type B timer)
    while (count--)
    {
        TMR1 = 0;
        while (TMR1 < 0x4e)
            ;
    }
    T1CONbits.ON = 0;
}

void __ISR(_DMA_0_VECTOR, ipl7AUTO) _IntHandlerSysDmaCh0(void)
{
    bufferAFull = 0;
    DCH1CONSET = 0x0000080;
    DCH0INTCLR = 0x0000ff;
    IFS1CLR = (1 << 28);
}

void __ISR(_DMA_1_VECTOR, ipl7AUTO) _IntHandlerSysDmaCh1(void)
{
    bufferBFull = 0;
    DCH0CONSET = 0x00000080;
    DCH1INTCLR = 0x0000ff;
    IFS1CLR = (1 << 29);
}